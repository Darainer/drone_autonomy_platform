#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <drone_autonomy_msgs/msg/attitude_command.hpp>

class CommunicationNode : public rclcpp::Node
{
public:
    CommunicationNode() : Node("communication_node")
    {
        attitude_command_subscriber_ = this->create_subscription<drone_autonomy_msgs::msg::AttitudeCommand>(
            "~/attitude_command", 10, std::bind(&CommunicationNode::attitudeCommandCallback, this, std::placeholders::_1));

        mavros_state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, std::bind(&CommunicationNode::mavrosStateCallback, this, std::placeholders::_1));
    }

private:
    void attitudeCommandCallback(const drone_autonomy_msgs::msg::AttitudeCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received attitude command: roll=%f, pitch=%f, yaw=%f, thrust=%f",
            msg->roll, msg->pitch, msg->yaw, msg->thrust);
    }

    void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received MAVROS state: connected=%d, armed=%d, mode=%s",
            msg->connected, msg->armed, msg->mode.c_str());
    }

    rclcpp::Subscription<drone_autonomy_msgs::msg::AttitudeCommand>::SharedPtr attitude_command_subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommunicationNode>());
    rclcpp::shutdown();
    return 0;
}
