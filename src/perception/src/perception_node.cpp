#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <drone_autonomy_msgs/msg/sensor_data.hpp>

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
        // Publishers
        sensor_data_pub_ = this->create_publisher<drone_autonomy_msgs::msg::SensorData>(
            "~/sensor_data", rclcpp::SensorDataQoS());

        // Subscribers — OAK-D publishes on /oak/rgb/image_raw by default
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oak/rgb/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&PerceptionNode::imageCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oak/stereo/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&PerceptionNode::depthCallback, this, std::placeholders::_1));

        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/detections", 10,
            std::bind(&PerceptionNode::detectionCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Perception node started — waiting for OAK-D data");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        last_image_ = msg;
        publishSensorData();
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        last_depth_ = msg;
        RCLCPP_DEBUG(this->get_logger(), "Depth frame: %ux%u", msg->width, msg->height);
    }

    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detections: %zu objects", msg->detections.size());
    }

    void publishSensorData()
    {
        if (!last_image_) return;

        auto sensor_msg = drone_autonomy_msgs::msg::SensorData();
        sensor_msg.image = *last_image_;
        sensor_data_pub_->publish(sensor_msg);
    }

    rclcpp::Publisher<drone_autonomy_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

    sensor_msgs::msg::Image::SharedPtr last_image_;
    sensor_msgs::msg::Image::SharedPtr last_depth_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
