#include <rclcpp/rclcpp.hpp>

#include "autonomy/autonomy_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomyNode>());
    rclcpp::shutdown();
    return 0;
}
