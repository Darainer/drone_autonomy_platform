#include <rclcpp/rclcpp.hpp>
#include <drone_autonomy_msgs/msg/mission.hpp>

class AutonomyNode : public rclcpp::Node
{
public:
    AutonomyNode() : Node("autonomy_node")
    {
        mission_publisher_ = this->create_publisher<drone_autonomy_msgs::msg::Mission>("~/mission", 10);
    }

    void publish_mission()
    {
        auto mission = drone_autonomy_msgs::msg::Mission();
        mission.mission_id = "test_mission";
        mission.mission_type = "waypoints";
        mission_publisher_->publish(mission);
    }

private:
    rclcpp::Publisher<drone_autonomy_msgs::msg::Mission>::SharedPtr mission_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomyNode>();
    node->publish_mission();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
