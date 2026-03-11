// Pull in the node class; rename its main() so GTest's main (from gtest_main) wins.
#define main _battery_monitor_main
#include "../src/battery_monitor.cpp"
#undef main

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class BatteryMonitorTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()    { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    mode_call_count_ = 0;
    last_mode_.clear();

    test_node_ = rclcpp::Node::make_shared("bm_test_helper");
    monitor_   = std::make_shared<safety::BatteryMonitor>();

    // Fake /mavros/set_mode service — records calls made by BatteryMonitor
    set_mode_svc_ = test_node_->create_service<mavros_msgs::srv::SetMode>(
      "/mavros/set_mode",
      [this](const mavros_msgs::srv::SetMode::Request::SharedPtr req,
             mavros_msgs::srv::SetMode::Response::SharedPtr resp) {
        std::lock_guard<std::mutex> lock(mu_);
        last_mode_ = req->custom_mode;
        mode_call_count_++;
        resp->mode_sent = true;
      });

    battery_pub_ = test_node_->create_publisher<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", 10);

    executor_.add_node(test_node_);
    executor_.add_node(monitor_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(test_node_);
    executor_.remove_node(monitor_);
    monitor_.reset();
    test_node_.reset();
  }

  void publish_battery(float percentage)
  {
    sensor_msgs::msg::BatteryState msg;
    msg.percentage = percentage;
    battery_pub_->publish(msg);
  }

  int call_count()
  {
    std::lock_guard<std::mutex> lock(mu_);
    return mode_call_count_;
  }

  std::string called_mode()
  {
    std::lock_guard<std::mutex> lock(mu_);
    return last_mode_;
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<safety::BatteryMonitor> monitor_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_svc_;

  std::mutex mu_;
  int mode_call_count_{0};
  std::string last_mode_;
  std::thread spin_thread_;
};

// Normal battery (80%) — no service call
TEST_F(BatteryMonitorTest, NormalBatteryNoAction)
{
  publish_battery(0.80f);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(call_count(), 0);
}

// Warning zone (20%: between 15–25%) — no RTL
TEST_F(BatteryMonitorTest, WarningZoneNoRTL)
{
  publish_battery(0.20f);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(call_count(), 0);
}

// Exactly at warning threshold (25%) — no RTL
TEST_F(BatteryMonitorTest, ExactlyAtWarningThresholdNoRTL)
{
  publish_battery(0.25f);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(call_count(), 0);
}

// Critical (10%) — triggers RTL with correct mode string
TEST_F(BatteryMonitorTest, CriticalBatteryTriggersRTL)
{
  publish_battery(0.10f);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(call_count(), 1);
  EXPECT_EQ(called_mode(), "AUTO.RTL");
}

// Exactly at critical threshold (15%) — triggers RTL
TEST_F(BatteryMonitorTest, ExactlyAtCriticalThresholdTriggersRTL)
{
  publish_battery(0.15f);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(call_count(), 1);
  EXPECT_EQ(called_mode(), "AUTO.RTL");
}

// Just above critical (16%) — no RTL
TEST_F(BatteryMonitorTest, JustAboveCriticalNoRTL)
{
  publish_battery(0.16f);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(call_count(), 0);
}

// RTL fires only once even with repeated critical messages (rtl_triggered_ guard)
TEST_F(BatteryMonitorTest, RTLTriggeredOnlyOnce)
{
  publish_battery(0.10f);
  std::this_thread::sleep_for(200ms);
  publish_battery(0.05f);
  std::this_thread::sleep_for(200ms);
  publish_battery(0.00f);
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(call_count(), 1);
}

// Negative percentage — driver reports "not available", must be ignored
TEST_F(BatteryMonitorTest, NegativePercentageIgnored)
{
  publish_battery(-1.0f);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(call_count(), 0);
}

// Sequence: normal → warn → critical
TEST_F(BatteryMonitorTest, DrainSequenceTriggersRTLOnce)
{
  publish_battery(0.60f);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(call_count(), 0);

  publish_battery(0.20f);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(call_count(), 0);

  publish_battery(0.10f);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(call_count(), 1);
  EXPECT_EQ(called_mode(), "AUTO.RTL");
}
