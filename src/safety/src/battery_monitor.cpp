#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

namespace safety {

static constexpr float BATTERY_WARN_PCT    = 0.25f;  // 25% — log warning
static constexpr float BATTERY_CRITICAL_PCT = 0.15f; // 15% — trigger RTL

class BatteryMonitor : public rclcpp::Node
{
public:
    BatteryMonitor() : Node("battery_monitor")
    {
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", 10,
            std::bind(&BatteryMonitor::batteryCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Battery Monitor started (warn=%.0f%% critical=%.0f%%)",
                    BATTERY_WARN_PCT * 100.0f, BATTERY_CRITICAL_PCT * 100.0f);
    }

private:
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        const float pct = msg->percentage;

        if (pct < 0.0f) {
            // percentage not provided by this battery driver
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Battery: %.2f V  %.1f%%", msg->voltage, pct * 100.0f);

        if (pct <= BATTERY_CRITICAL_PCT && !rtl_triggered_) {
            RCLCPP_ERROR(this->get_logger(),
                         "CRITICAL battery at %.1f%% — commanding RTL", pct * 100.0f);
            triggerRTL();
        } else if (pct <= BATTERY_WARN_PCT) {
            RCLCPP_WARN(this->get_logger(), "Low battery: %.1f%%", pct * 100.0f);
        }
    }

    void triggerRTL()
    {
        if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "SetMode service unavailable — cannot trigger RTL");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "AUTO.RTL";

        set_mode_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                if (future.get()->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "RTL mode set successfully");
                    rtl_triggered_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set RTL mode");
                }
            });
    }

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    bool rtl_triggered_{false};
};

}  // namespace safety

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<safety::BatteryMonitor>());
    rclcpp::shutdown();
    return 0;
}
