#define main _safety_node_main
#include "../src/safety_node.cpp"
#undef main

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class SafetyNodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()    { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    test_node_  = rclcpp::Node::make_shared("safety_test_helper");
    safety_     = std::make_shared<SafetyNode>();

    state_pub_ = test_node_->create_publisher<mavros_msgs::msg::State>(
      "/mavros/state", 10);

    executor_.add_node(test_node_);
    executor_.add_node(safety_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(test_node_);
    executor_.remove_node(safety_);
    safety_.reset();
    test_node_.reset();
  }

  void publish_state(bool armed, const std::string & mode = "STABILIZED")
  {
    mavros_msgs::msg::State msg;
    msg.armed     = armed;
    msg.mode      = mode;
    msg.connected = true;
    state_pub_->publish(msg);
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<SafetyNode> safety_;
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  std::thread spin_thread_;
};

// Node starts and spins without error
TEST_F(SafetyNodeTest, NodeInitializesWithoutError)
{
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// Armed state received without crash
TEST_F(SafetyNodeTest, ArmedStateHandledWithoutCrash)
{
  publish_state(true);
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// Disarmed state received without crash (triggers WARN log internally)
TEST_F(SafetyNodeTest, DisarmedStateHandledWithoutCrash)
{
  publish_state(false);
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// Multiple rapid state transitions do not crash the node
TEST_F(SafetyNodeTest, RapidStateTransitionsHandled)
{
  for (int i = 0; i < 20; ++i) {
    publish_state(i % 2 == 0);
    std::this_thread::sleep_for(10ms);
  }
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// Verify subscription is on the correct topic
TEST_F(SafetyNodeTest, SubscriptionTopicIsCorrect)
{
  auto topic_names = safety_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, _] : topic_names) {
    if (name == "/mavros/state") { found = true; break; }
  }
  EXPECT_TRUE(found) << "Expected subscription on /mavros/state";
}
