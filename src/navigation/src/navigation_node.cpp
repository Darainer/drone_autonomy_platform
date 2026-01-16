#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <drone_autonomy_msgs/msg/mission.hpp>
#include <drone_autonomy_msgs/msg/trajectory.hpp>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node")
    {
        mission_subscriber_ = this->create_subscription<drone_autonomy_msgs::msg::Mission>(
            "~/mission", 10, std::bind(&NavigationNode::missionCallback, this, std::placeholders::_1));

        trajectory_publisher_ = this->create_publisher<drone_autonomy_msgs::msg::Trajectory>("~/trajectory", 10);
    }

private:
    void missionCallback(const drone_autonomy_msgs::msg::Mission::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received mission: %s", msg->mission_id.c_str());
        // In a real application, you would do some processing on the mission here
        // and then publish a trajectory
    }

    rclcpp::Subscription<drone_autonomy_msgs::msg::Mission>::SharedPtr mission_subscriber_;
    rclcpp::Publisher<drone_autonomy_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}
