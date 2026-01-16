#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode() : Node("safety_node")
    {
        mavros_state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, std::bind(&SafetyNode::mavrosStateCallback, this, std::placeholders::_1));
    }

private:
    void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if (!msg->armed)
        {
            RCLCPP_WARN(this->get_logger(), "Drone is disarmed!");
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}
