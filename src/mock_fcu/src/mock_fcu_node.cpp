/**
 * mock_fcu_node.cpp
 *
 * Simulates the MAVROS interface that the safety, control, and communication
 * nodes expect from a real Pixhawk flight controller. Use this in integration
 * tests and launch_testing suites instead of running a full PX4 SITL stack.
 *
 * Published topics (mirrors MAVROS output):
 *   /mavros/state      — mavros_msgs/State
 *   /mavros/battery    — sensor_msgs/BatteryState
 *
 * Subscribed topics (command injection for tests):
 *   /mock_fcu/set_battery_pct   — std_msgs/Float32  (0.0–1.0)
 *   /mock_fcu/set_armed         — std_msgs/Bool
 *   /mock_fcu/set_mode          — std_msgs/String
 *
 * Services (mirrors MAVROS services):
 *   /mavros/set_mode   — mavros_msgs/srv/SetMode
 *
 * Parameters:
 *   armed              (bool,   default false)
 *   mode               (string, default "STABILIZED")
 *   battery_percentage (double, default 1.0)
 *   publish_hz         (double, default 10.0)
 *   simulate_disarm_on_rtl (bool, default true)
 */

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

class MockFCU : public rclcpp::Node
{
public:
    MockFCU() : Node("mock_fcu")
    {
        // Parameters — tests can override via node options or yaml
        declare_parameter("armed",                  false);
        declare_parameter("mode",                   std::string("STABILIZED"));
        declare_parameter("battery_percentage",     1.0);
        declare_parameter("publish_hz",             10.0);
        declare_parameter("simulate_disarm_on_rtl", true);

        armed_      = get_parameter("armed").as_bool();
        mode_       = get_parameter("mode").as_string();
        battery_pct_= static_cast<float>(get_parameter("battery_percentage").as_double());
        disarm_on_rtl_ = get_parameter("simulate_disarm_on_rtl").as_bool();

        // Publishers (MAVROS topics consumed by platform nodes)
        state_pub_   = create_publisher<mavros_msgs::msg::State>("/mavros/state", 10);
        battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/mavros/battery", 10);

        // Service — mirrors /mavros/set_mode
        set_mode_svc_ = create_service<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode",
            [this](const mavros_msgs::srv::SetMode::Request::SharedPtr req,
                   mavros_msgs::srv::SetMode::Response::SharedPtr resp) {
                handleSetMode(req, resp);
            });

        // Injection topics — used by tests to drive state changes
        battery_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/mock_fcu/set_battery_pct", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mu_);
                battery_pct_ = msg->data;
            });

        armed_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mock_fcu/set_armed", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mu_);
                armed_ = msg->data;
            });

        mode_sub_ = create_subscription<std_msgs::msg::String>(
            "/mock_fcu/set_mode", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mu_);
                mode_ = msg->data;
            });

        double hz = get_parameter("publish_hz").as_double();
        auto period = std::chrono::duration<double>(1.0 / hz);
        timer_ = create_wall_timer(period, [this]() { publishAll(); });

        RCLCPP_INFO(get_logger(),
            "MockFCU ready — armed=%s mode=%s battery=%.0f%%",
            armed_ ? "true" : "false", mode_.c_str(), battery_pct_ * 100.0f);
    }

    // Accessors for test assertions
    int set_mode_call_count() const
    {
        std::lock_guard<std::mutex> lock(mu_);
        return set_mode_call_count_;
    }

    std::string last_requested_mode() const
    {
        std::lock_guard<std::mutex> lock(mu_);
        return last_requested_mode_;
    }

private:
    void handleSetMode(const mavros_msgs::srv::SetMode::Request::SharedPtr req,
                       mavros_msgs::srv::SetMode::Response::SharedPtr resp)
    {
        std::lock_guard<std::mutex> lock(mu_);
        last_requested_mode_ = req->custom_mode;
        set_mode_call_count_++;

        RCLCPP_INFO(get_logger(), "set_mode called: '%s'", req->custom_mode.c_str());

        if (req->custom_mode == "AUTO.RTL" && disarm_on_rtl_) {
            armed_ = false;
        }
        mode_ = req->custom_mode;
        resp->mode_sent = true;
    }

    void publishAll()
    {
        std::lock_guard<std::mutex> lock(mu_);

        mavros_msgs::msg::State state_msg;
        state_msg.header.stamp = now();
        state_msg.connected    = true;
        state_msg.armed        = armed_;
        state_msg.mode         = mode_;
        state_pub_->publish(state_msg);

        sensor_msgs::msg::BatteryState bat_msg;
        bat_msg.header.stamp = now();
        bat_msg.percentage   = battery_pct_;
        bat_msg.voltage      = battery_pct_ * 16.8f;  // simulate 4S LiPo
        battery_pub_->publish(bat_msg);
    }

    mutable std::mutex mu_;

    bool armed_{false};
    std::string mode_;
    float battery_pct_{1.0f};
    bool disarm_on_rtl_{true};
    int set_mode_call_count_{0};
    std::string last_requested_mode_;

    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_svc_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockFCU>());
    rclcpp::shutdown();
    return 0;
}
