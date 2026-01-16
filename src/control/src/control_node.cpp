#include <rclcpp/rclcpp.hpp>
#include <drone_autonomy_msgs/msg/trajectory.hpp>
#include <drone_autonomy_msgs/msg/attitude_command.hpp>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        trajectory_subscriber_ = this->create_subscription<drone_autonomy_msgs::msg::Trajectory>(
            "~/trajectory", 10, std::bind(&ControlNode::trajectoryCallback, this, std::placeholders::_1));

        attitude_command_publisher_ = this->create_publisher<drone_autonomy_msgs::msg::AttitudeCommand>("~/attitude_command", 10);
    }

private:
    void trajectoryCallback(const drone_autonomy_msgs::msg::Trajectory::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received trajectory");
        // In a real application, you would do some processing on the trajectory here
        // and then publish an attitude command
    }

    rclcpp::Subscription<drone_autonomy_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
    rclcpp::Publisher<drone_autonomy_msgs::msg::AttitudeCommand>::SharedPtr attitude_command_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
