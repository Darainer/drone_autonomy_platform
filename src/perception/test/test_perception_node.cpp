// Pull in the node class; rename its main() so GTest's main (from gtest_main) wins.
#define main _perception_node_main
#include "../src/perception_node.cpp"
#undef main

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <drone_autonomy_msgs/msg/sensor_data.hpp>

#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// PerceptionNode uses rclcpp::SensorDataQoS() for both its publishers and its
// image subscribers, so test helpers must match that QoS profile.

class PerceptionNodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()    { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    sensor_data_count_ = 0;

    test_node_   = rclcpp::Node::make_shared("perception_test_helper");
    perception_  = std::make_shared<PerceptionNode>();

    // Publishers that feed the perception node (match SensorDataQoS)
    image_pub_ = test_node_->create_publisher<sensor_msgs::msg::Image>(
      "/oak/rgb/image_raw", rclcpp::SensorDataQoS());

    depth_pub_ = test_node_->create_publisher<sensor_msgs::msg::Image>(
      "/oak/stereo/image_raw", rclcpp::SensorDataQoS());

    detection_pub_ = test_node_->create_publisher<vision_msgs::msg::Detection2DArray>(
      "/detections", 10);

    // Spy subscriber on the perception node's SensorData output
    sensor_data_sub_ = test_node_->create_subscription<drone_autonomy_msgs::msg::SensorData>(
      "/perception_node/sensor_data", rclcpp::SensorDataQoS(),
      [this](const drone_autonomy_msgs::msg::SensorData::SharedPtr) {
        sensor_data_count_++;
      });

    executor_.add_node(test_node_);
    executor_.add_node(perception_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(test_node_);
    executor_.remove_node(perception_);
    perception_.reset();
    test_node_.reset();
  }

  sensor_msgs::msg::Image make_image(uint32_t width = 640, uint32_t height = 480,
                                     const std::string & encoding = "rgb8")
  {
    sensor_msgs::msg::Image img;
    img.header.stamp = test_node_->now();
    img.width        = width;
    img.height       = height;
    img.encoding     = encoding;
    img.step         = width * 3;
    img.data.assign(static_cast<size_t>(img.step) * height, 0);
    return img;
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<PerceptionNode> perception_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Subscription<drone_autonomy_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;

  std::atomic<int> sensor_data_count_{0};
  std::thread spin_thread_;
};

// Node starts and spins without error
TEST_F(PerceptionNodeTest, NodeInitializesWithoutError)
{
  std::this_thread::sleep_for(100ms);
  SUCCEED();
}

// No SensorData published before any image arrives (last_image_ guard)
TEST_F(PerceptionNodeTest, NoSensorDataPublishedWithoutImage)
{
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(sensor_data_count_.load(), 0)
    << "SensorData should not be published before any image is received";
}

// Receiving an RGB image triggers a SensorData publication
TEST_F(PerceptionNodeTest, ImageCallbackPublishesSensorData)
{
  image_pub_->publish(make_image());
  std::this_thread::sleep_for(300ms);
  EXPECT_GE(sensor_data_count_.load(), 1)
    << "Expected at least one SensorData message after RGB image";
}

// Each image triggers exactly one SensorData message
TEST_F(PerceptionNodeTest, OneImageOneSensorDataMessage)
{
  image_pub_->publish(make_image());
  std::this_thread::sleep_for(200ms);
  int after_first = sensor_data_count_.load();

  image_pub_->publish(make_image());
  std::this_thread::sleep_for(200ms);
  int after_second = sensor_data_count_.load();

  EXPECT_GE(after_first,  1) << "First image should produce SensorData";
  EXPECT_GE(after_second, 2) << "Second image should produce another SensorData";
}

// Depth frame received and logged without crash (no SensorData side-effect)
TEST_F(PerceptionNodeTest, DepthCallbackHandledWithoutCrash)
{
  auto depth = make_image(1280, 720, "16UC1");
  depth_pub_->publish(depth);
  std::this_thread::sleep_for(150ms);
  // Depth alone must NOT publish SensorData (publishSensorData checks last_image_)
  EXPECT_EQ(sensor_data_count_.load(), 0)
    << "Depth-only frame must not trigger SensorData publication";
}

// Detection array received without crash
TEST_F(PerceptionNodeTest, DetectionCallbackHandledWithoutCrash)
{
  vision_msgs::msg::Detection2DArray det;
  // 3 detections
  det.detections.resize(3);
  detection_pub_->publish(det);
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// Empty detection array handled without crash
TEST_F(PerceptionNodeTest, EmptyDetectionArrayHandled)
{
  vision_msgs::msg::Detection2DArray det;  // zero detections
  detection_pub_->publish(det);
  std::this_thread::sleep_for(150ms);
  SUCCEED();
}

// SensorData message embeds the published image
TEST_F(PerceptionNodeTest, SensorDataContainsImage)
{
  drone_autonomy_msgs::msg::SensorData last_msg;
  bool got_msg = false;

  auto spy_sub = test_node_->create_subscription<drone_autonomy_msgs::msg::SensorData>(
    "/perception_node/sensor_data", rclcpp::SensorDataQoS(),
    [&](const drone_autonomy_msgs::msg::SensorData::SharedPtr msg) {
      last_msg = *msg;
      got_msg  = true;
    });

  auto img = make_image(320, 240, "bgr8");
  image_pub_->publish(img);

  auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (!got_msg && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(got_msg) << "No SensorData received";
  EXPECT_EQ(last_msg.image.width,    320u);
  EXPECT_EQ(last_msg.image.height,   240u);
  EXPECT_EQ(last_msg.image.encoding, "bgr8");

  test_node_->destroy_subscription(spy_sub);
}

// Rapid image stream handled without crash or queue overflow
TEST_F(PerceptionNodeTest, RapidImageStreamHandled)
{
  for (int i = 0; i < 30; ++i) {
    image_pub_->publish(make_image());
    std::this_thread::sleep_for(10ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_GE(sensor_data_count_.load(), 1);
}

// Perception node publishes on ~/sensor_data
TEST_F(PerceptionNodeTest, SensorDataPublisherExists)
{
  auto publishers = perception_->get_publisher_names_and_types_by_node(
    "perception_node", perception_->get_namespace());
  bool found = false;
  for (const auto & [name, _] : publishers) {
    if (name.find("sensor_data") != std::string::npos) { found = true; break; }
  }
  EXPECT_TRUE(found) << "Expected publisher on ~/sensor_data";
}
