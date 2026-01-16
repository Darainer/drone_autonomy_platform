#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <rclcpp/rclcpp.hpp>

namespace common
{

inline void print_hello()
{
    RCLCPP_INFO(rclcpp::get_logger("common"), "Hello from common!");
}

}  // namespace common

#endif  // COMMON_HPP_
