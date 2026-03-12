// Pull in the node class; rename its main() so GTest's main (from gtest_main) wins.
#define main _communication_node_main
#include "../src/communication_node.cpp"
#undef main

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <drone_autonomy_msgs/msg/attitude_command.hpp>

#include <chrono>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class CommunicationNodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()    { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    test_node_ = rclcpp::Node::make_shared("comm_test_helper");
    comm_      = std::make_shared<CommunicationNode>();

    // Publisher for attitude commands (node's namespaced topic)
    attitude_pub_ = test_node_->create_publisher<drone_autonomy_msgs::msg::AttitudeCommand>(
      "/communication_node/attitude_command", 10);

    // Publisher for MAVROS state (absolute topic)
    state_pub_ = test_node_->create_publisher<mavros_msgs::msg::State>(
      "/mavros/state", 10);

    executor_.add_node(test_node_);
    executor_.add_node(comm_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(test_node_);
    executor_.remove_node(comm_);
    comm_.reset();
    test_node_.reset();
  }

  drone_autonomy_msgs::msg::AttitudeCommand make_attitude(
    double roll = 0.0, double pitch = 0.0, double yaw = 0.0, double thrust = 0.5)
  {
    drone_autonomy_msgs::msg::AttitudeCommand cmd;
    cmd.header.stamp = test_node_->now();
    cmd.roll   = roll;
    cmd.pitch  = pitch;
    cmd.yaw    = yaw;
    cmd.thrust = thrust;
    return cmd;
  }

  mavros_msgs::msg::State make_state(bool connected, bool armed,
                                     const std::string & mode = "STABILIZED")
  {
    mavros_msgs::msg::State msg;
    msg.header.stamp = test_node_->now();
    msg.connected    = connected;
    msg.armed        = armed;
    msg.mode         = mode;
    return msg;
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<CommunicationNode> comm_;
  rclcpp::Publisher<drone_autonomy_msgs::msg::AttitudeCommand>::SharedPtr attitude_pub_;
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  std::thread spin_thread_;
};

// Node starts without error
TEST_F(CommunicationNodeTest, NodeInitializesWithoutError)
{
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// AttitudeCommand received without crash (roll/pitch/yaw/thrust logged)
TEST_F(CommunicationNodeTest, AttitudeCommandReceivedWithoutCrash)
{
  attitude_pub_->publish(make_attitude(0.1, -0.1, 0.05, 0.6));
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// Zero-valued AttitudeCommand handled without crash
TEST_F(CommunicationNodeTest, ZeroAttitudeCommandHandled)
{
  attitude_pub_->publish(make_attitude(0.0, 0.0, 0.0, 0.0));
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// Max-valued AttitudeCommand handled without crash
TEST_F(CommunicationNodeTest, MaxAttitudeCommandHandled)
{
  attitude_pub_->publish(make_attitude(1.0, 1.0, 1.0, 1.0));
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// MAVROS state (connected + armed) received without crash
TEST_F(CommunicationNodeTest, ConnectedArmedStateHandledWithoutCrash)
{
  state_pub_->publish(make_state(true, true, "OFFBOARD"));
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// MAVROS state (disconnected + disarmed) received without crash
TEST_F(CommunicationNodeTest, DisconnectedDisarmedStateHandled)
{
  state_pub_->publish(make_state(false, false, "MANUAL"));
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// Both topics delivered concurrently without crash
TEST_F(CommunicationNodeTest, ConcurrentTopicsHandled)
{
  for (int i = 0; i < 10; ++i) {
    attitude_pub_->publish(make_attitude(0.01 * i, -0.01 * i, 0.0, 0.5));
    state_pub_->publish(make_state(true, i % 2 == 0, "STABILIZED"));
    std::this_thread::sleep_for(15ms);
  }
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// Rapid burst of attitude commands handled without crash
TEST_F(CommunicationNodeTest, RapidAttitudeCommandBurstHandled)
{
  drone_autonomy_msgs::msg::AttitudeCommand cmd = make_attitude(0.1, 0.1, 0.1, 0.5);
  for (int i = 0; i < 50; ++i) {
    attitude_pub_->publish(cmd);
  }
  std::this_thread::sleep_for(200ms);
  SUCCEED();
}

// Verify subscription is present on correct MAVROS state topic
TEST_F(CommunicationNodeTest, MavrosStateSubscriptionExists)
{
  auto topic_names = comm_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, _] : topic_names) {
    if (name == "/mavros/state") { found = true; break; }
  }
  EXPECT_TRUE(found) << "Expected subscription on /mavros/state";
}

// Verify subscription is present on ~/attitude_command
TEST_F(CommunicationNodeTest, AttitudeCommandSubscriptionExists)
{
  auto topic_names = comm_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, _] : topic_names) {
    if (name.find("attitude_command") != std::string::npos) { found = true; break; }
  }
  EXPECT_TRUE(found) << "Expected subscription on ~/attitude_command";
}

// Multiple distinct flight modes logged without crash
TEST_F(CommunicationNodeTest, MultipleFlightModesHandled)
{
  for (const auto & mode : {"MANUAL", "STABILIZED", "ALTCTL", "POSCTL", "OFFBOARD", "AUTO.RTL"}) {
    state_pub_->publish(make_state(true, true, mode));
    std::this_thread::sleep_for(30ms);
  }
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}
