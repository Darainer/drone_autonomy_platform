#define main _navigation_node_main
#include "../src/navigation_node.cpp"
#undef main

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <drone_autonomy_msgs/msg/mission.hpp>
#include <drone_autonomy_msgs/msg/trajectory.hpp>

#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class NavigationNodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()    { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    test_node_ = rclcpp::Node::make_shared("nav_test_helper");
    nav_       = std::make_shared<NavigationNode>();

    mission_pub_ = test_node_->create_publisher<drone_autonomy_msgs::msg::Mission>(
      "/navigation_node/mission", 10);

    traj_sub_ = test_node_->create_subscription<drone_autonomy_msgs::msg::Trajectory>(
      "/navigation_node/trajectory", 10,
      [this](const drone_autonomy_msgs::msg::Trajectory::SharedPtr) {
        received_trajectory_count_++;
      });

    executor_.add_node(test_node_);
    executor_.add_node(nav_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(test_node_);
    executor_.remove_node(nav_);
    nav_.reset();
    test_node_.reset();
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<NavigationNode> nav_;
  rclcpp::Publisher<drone_autonomy_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Subscription<drone_autonomy_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  std::atomic<int> received_trajectory_count_{0};
  std::thread spin_thread_;
};

// Node starts without error
TEST_F(NavigationNodeTest, NodeInitializesWithoutError)
{
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// Node has a publisher on ~/trajectory
TEST_F(NavigationNodeTest, TrajectoryPublisherExists)
{
  auto publishers = nav_->get_publisher_names_and_types_by_node(
    "navigation_node", nav_->get_namespace());
  bool found = false;
  for (const auto & [name, _] : publishers) {
    if (name.find("trajectory") != std::string::npos) { found = true; break; }
  }
  EXPECT_TRUE(found) << "Expected publisher on ~/trajectory";
}

// Mission with a valid ID is received without crash
TEST_F(NavigationNodeTest, MissionReceivedWithoutCrash)
{
  drone_autonomy_msgs::msg::Mission msg;
  msg.mission_id = "test-mission-001";
  mission_pub_->publish(msg);
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// Empty mission ID handled without crash
TEST_F(NavigationNodeTest, EmptyMissionIdHandled)
{
  drone_autonomy_msgs::msg::Mission msg;
  msg.mission_id = "";
  mission_pub_->publish(msg);
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// Multiple missions handled in sequence
TEST_F(NavigationNodeTest, MultipleMissionsHandled)
{
  for (int i = 0; i < 5; ++i) {
    drone_autonomy_msgs::msg::Mission msg;
    msg.mission_id = "mission-" + std::to_string(i);
    mission_pub_->publish(msg);
    std::this_thread::sleep_for(30ms);
  }
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}
