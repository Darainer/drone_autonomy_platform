#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <isaac_ros_dnn_inference/msg/dnn_inference.hpp>
#include <drone_autonomy_msgs/msg/sensor_data.hpp>

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
        sensor_data_publisher_ = this->create_publisher<drone_autonomy_msgs::msg::SensorData>("~/sensor_data", 10);

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&PerceptionNode::imageCallback, this, std::placeholders::_1));

        visual_slam_status_subscriber_ = this->create_subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>(
            "/visual_slam/status", 10, std::bind(&PerceptionNode::visualSlamStatusCallback, this, std::placeholders::_1));

        dnn_inference_subscriber_ = this->create_subscription<isaac_ros_dnn_inference::msg::DnnInference>(
            "/dnn_inference", 10, std::bind(&PerceptionNode::dnnInferenceCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image");
        // In a real application, you would do some processing on the image here
    }

    void visualSlamStatusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Visual SLAM status");
    }

    void dnnInferenceCallback(const isaac_ros_dnn_inference::msg::DnnInference::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received DNN inference");
    }

    rclcpp::Publisher<drone_autonomy_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr visual_slam_status_subscriber_;
    rclcpp::Subscription<isaac_ros_dnn_inference::msg::DnnInference>::SharedPtr dnn_inference_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
