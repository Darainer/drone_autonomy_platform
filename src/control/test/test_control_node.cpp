#define main _control_node_main
#include "../src/control_node.cpp"
#undef main

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <drone_autonomy_msgs/msg/trajectory.hpp>
#include <drone_autonomy_msgs/msg/attitude_command.hpp>

#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class ControlNodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()    { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    received_trajectory_count_ = 0;

    test_node_ = rclcpp::Node::make_shared("control_test_helper");
    control_   = std::make_shared<ControlNode>();

    // Publish trajectory to the node's namespaced topic
    traj_pub_ = test_node_->create_publisher<drone_autonomy_msgs::msg::Trajectory>(
      "/control_node/trajectory", 10);

    // Spy subscriber on attitude_command output
    attitude_sub_ = test_node_->create_subscription<drone_autonomy_msgs::msg::AttitudeCommand>(
      "/control_node/attitude_command", 10,
      [this](const drone_autonomy_msgs::msg::AttitudeCommand::SharedPtr) {
        received_trajectory_count_++;
      });

    executor_.add_node(test_node_);
    executor_.add_node(control_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(test_node_);
    executor_.remove_node(control_);
    control_.reset();
    test_node_.reset();
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<ControlNode> control_;
  rclcpp::Publisher<drone_autonomy_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Subscription<drone_autonomy_msgs::msg::AttitudeCommand>::SharedPtr attitude_sub_;
  std::atomic<int> received_trajectory_count_{0};
  std::thread spin_thread_;
};

// Node initializes without error
TEST_F(ControlNodeTest, NodeInitializesWithoutError)
{
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// Node has a publisher on ~/attitude_command
TEST_F(ControlNodeTest, AttitudeCommandPublisherExists)
{
  auto publishers = control_->get_publisher_names_and_types_by_node(
    "control_node", control_->get_namespace());
  bool found = false;
  for (const auto & [name, _] : publishers) {
    if (name.find("attitude_command") != std::string::npos) { found = true; break; }
  }
  EXPECT_TRUE(found) << "Expected publisher on ~/attitude_command";
}

// Trajectory message is received without crashing
TEST_F(ControlNodeTest, TrajectoryReceivedWithoutCrash)
{
  drone_autonomy_msgs::msg::Trajectory msg;
  traj_pub_->publish(msg);
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// Multiple trajectory messages handled without crash
TEST_F(ControlNodeTest, MultipleTrajectoriesHandled)
{
  drone_autonomy_msgs::msg::Trajectory msg;
  for (int i = 0; i < 10; ++i) {
    traj_pub_->publish(msg);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}
