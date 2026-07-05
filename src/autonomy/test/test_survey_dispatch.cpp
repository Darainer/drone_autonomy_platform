// TP-002 TS-01 / TS-02 — ROS-graph tests for autonomy_node survey dispatch
// and rejection (DES-003 autonomy_node behavior, items 1-3).
//
// Verifies: MAP-6
//
// Each test spins up the real AutonomyNode plus a harness node that
// publishes on ~/survey_request and subscribes ~/mission + ~/mission_status,
// then asserts on what came back. This is a ROS-graph/SITL-style test per
// TP-002 ("Environment: SITL / ROS graph with autonomy_node only") — it
// requires a full rclcpp + colcon build and is expected to run in CI, not in
// a sandbox without ROS2 installed.

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <drone_autonomy_msgs/msg/mission.hpp>
#include <drone_autonomy_msgs/msg/mission_status.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "autonomy/autonomy_node.hpp"

using drone_autonomy_msgs::msg::Mission;
using drone_autonomy_msgs::msg::MissionStatus;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Polygon;

namespace
{

Point32 makePoint(float x, float y)
{
    Point32 p;
    p.x = x;
    p.y = y;
    p.z = 0.0f;
    return p;
}

// TP-002 F1 REF-RECT: 100 m x 80 m rectangle.
Polygon refRect()
{
    Polygon poly;
    poly.points = {makePoint(0, 0), makePoint(100, 0), makePoint(100, 80), makePoint(0, 80)};
    return poly;
}

// TP-002 F1 REF-BAD-*: 2-vertex degenerate polygon.
Polygon refBadTwoVertices()
{
    Polygon poly;
    poly.points = {makePoint(0, 0), makePoint(10, 0)};
    return poly;
}

// TP-002 F1 REF-BAD-*: self-intersecting bowtie (edges 0-1 and 2-3 cross).
Polygon refBadBowtie()
{
    Polygon poly;
    poly.points = {makePoint(0, 0), makePoint(10, 10), makePoint(10, 0), makePoint(0, 10)};
    return poly;
}

// Simple (non-self-intersecting) concave "dart"/arrowhead quad: the final
// vertex is a reflex point pulled inside the hull of the other three, so the
// polygon is valid and non-self-intersecting yet non-convex — must be
// rejected by DES-003 D5 (v1 convex-only), not by the self-intersection rule.
Polygon concaveDart()
{
    Polygon poly;
    poly.points = {makePoint(0, 0), makePoint(100, 0), makePoint(50, 40), makePoint(50, 10)};
    return poly;
}

// Three collinear vertices: a valid vertex count and non-self-intersecting,
// but zero enclosed area — the coverage planner has nothing to sweep, so it
// must be rejected as degenerate before dispatch.
Polygon collinearPolygon()
{
    Polygon poly;
    poly.points = {makePoint(0, 0), makePoint(50, 0), makePoint(100, 0)};
    return poly;
}

// TP-002 "common parameters": altitude 40 m, forward 0.75, side 0.60,
// speed 5 m/s, capture_rate 2 Hz — all inside every DES-003 bound and
// consistent (s_cap*rate ~= 20.8 m/s at 40 m/55 deg VFOV >> 5 m/s).
Mission commonSurveyRequest(const Polygon & polygon)
{
    Mission m;
    m.mission_id = "req-1";
    m.mission_type = "survey";
    m.survey_polygon = polygon;
    m.survey_altitude_m = 40.0f;
    m.forward_overlap = 0.75f;
    m.side_overlap = 0.60f;
    m.survey_speed_ms = 5.0f;
    m.capture_rate_hz = 2.0f;
    return m;
}

}  // namespace

class SurveyDispatchTest : public ::testing::Test
{
protected:
    // ament_add_gtest links its own gtest_main, so this target must NOT define
    // its own main(). Initialize/shut down ROS once for the whole suite here.
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        autonomy_node_ = std::make_shared<AutonomyNode>();
        harness_node_ = std::make_shared<rclcpp::Node>("survey_test_harness");

        // AutonomyNode's own node name is "autonomy_node" and this test runs
        // it with no launch-file remaps, so its private topics resolve under
        // that namespace.
        request_publisher_ = harness_node_->create_publisher<Mission>(
            "/autonomy_node/survey_request", rclcpp::QoS(10).reliable());

        mission_subscriber_ = harness_node_->create_subscription<Mission>(
            "/autonomy_node/mission", rclcpp::QoS(10).reliable(),
            [this](Mission::SharedPtr msg) { missions_.push_back(*msg); });

        rclcpp::QoS status_qos(10);
        status_qos.reliable();
        status_qos.transient_local();
        status_subscriber_ = harness_node_->create_subscription<MissionStatus>(
            "/autonomy_node/mission_status", status_qos,
            [this](MissionStatus::SharedPtr msg) { statuses_.push_back(*msg); });

        executor_.add_node(autonomy_node_);
        executor_.add_node(harness_node_);
    }

    void spinFor(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some(std::chrono::milliseconds(20));
        }
    }

    // Publishes `request` and spins until a MissionStatus arrives or
    // `timeout` elapses, then drains a little more so a second, unexpected
    // publication would also be observed.
    void publishAndWait(const Mission & request,
                         std::chrono::milliseconds timeout = std::chrono::seconds(5))
    {
        // Let reliable/transient_local discovery complete before publishing.
        spinFor(std::chrono::milliseconds(200));
        request_publisher_->publish(request);

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (statuses_.empty() && std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some(std::chrono::milliseconds(50));
        }
        spinFor(std::chrono::milliseconds(200));
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<AutonomyNode> autonomy_node_;
    std::shared_ptr<rclcpp::Node> harness_node_;
    rclcpp::Publisher<Mission>::SharedPtr request_publisher_;
    rclcpp::Subscription<Mission>::SharedPtr mission_subscriber_;
    rclcpp::Subscription<MissionStatus>::SharedPtr status_subscriber_;
    std::vector<Mission> missions_;
    std::vector<MissionStatus> statuses_;
};

// TS-01 — Survey mission dispatch (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, ValidRequestDispatchesMissionAndActiveStatus)
{
    const auto request = commonSurveyRequest(refRect());
    publishAndWait(request);

    ASSERT_EQ(missions_.size(), 1u);
    EXPECT_EQ(missions_[0].mission_type, "survey");
    EXPECT_FALSE(missions_[0].mission_id.empty());
    EXPECT_NE(missions_[0].mission_id, request.mission_id);  // node mints its own id

    // "all spec fields byte-identical to the request" (TS-01 pass criterion).
    ASSERT_EQ(missions_[0].survey_polygon.points.size(), request.survey_polygon.points.size());
    for (size_t i = 0; i < request.survey_polygon.points.size(); ++i) {
        EXPECT_FLOAT_EQ(missions_[0].survey_polygon.points[i].x, request.survey_polygon.points[i].x);
        EXPECT_FLOAT_EQ(missions_[0].survey_polygon.points[i].y, request.survey_polygon.points[i].y);
    }
    EXPECT_FLOAT_EQ(missions_[0].survey_altitude_m, request.survey_altitude_m);
    EXPECT_FLOAT_EQ(missions_[0].forward_overlap, request.forward_overlap);
    EXPECT_FLOAT_EQ(missions_[0].side_overlap, request.side_overlap);
    EXPECT_FLOAT_EQ(missions_[0].survey_speed_ms, request.survey_speed_ms);
    EXPECT_FLOAT_EQ(missions_[0].capture_rate_hz, request.capture_rate_hz);

    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "active");
    EXPECT_EQ(statuses_[0].mission_id, missions_[0].mission_id);
}

// TS-02 — Degenerate survey rejection: REF-BAD, 2 vertices (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsTwoVertexPolygon)
{
    auto request = commonSurveyRequest(refBadTwoVertices());
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("vertices"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — Degenerate survey rejection: REF-BAD, self-intersecting bowtie
// (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsSelfIntersectingPolygon)
{
    auto request = commonSurveyRequest(refBadBowtie());
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("intersect"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — non-convex (concave) polygon, DES-003 D5 v1 convex-only
// (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsConcavePolygon)
{
    auto request = commonSurveyRequest(concaveDart());
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    // Rejected for convexity, not misclassified as self-intersecting.
    EXPECT_NE(statuses_[0].detail.find("convex"), std::string::npos) << statuses_[0].detail;
    EXPECT_EQ(statuses_[0].detail.find("intersect"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — altitude below the 10 m minimum (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsAltitudeTooLow)
{
    auto request = commonSurveyRequest(refRect());
    request.survey_altitude_m = 5.0f;
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("altitude"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — altitude above the 120 m maximum (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsAltitudeTooHigh)
{
    auto request = commonSurveyRequest(refRect());
    request.survey_altitude_m = 200.0f;
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("altitude"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — forward_overlap below the 0.30 minimum (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsForwardOverlapTooLow)
{
    auto request = commonSurveyRequest(refRect());
    request.forward_overlap = 0.1f;
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("overlap"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — side_overlap above the 0.95 maximum (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsSideOverlapTooHigh)
{
    auto request = commonSurveyRequest(refRect());
    request.side_overlap = 0.99f;
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("overlap"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — capture_rate_hz below the 0.5 Hz minimum (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsCaptureRateTooLow)
{
    auto request = commonSurveyRequest(refRect());
    request.capture_rate_hz = 0.1f;
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("capture_rate"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — capture_rate_hz above the 10 Hz maximum (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsCaptureRateTooHigh)
{
    auto request = commonSurveyRequest(refRect());
    request.capture_rate_hz = 20.0f;
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("capture_rate"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — speed/capture-rate inconsistency: 12 m/s at 0.5 Hz, which violates
// survey_speed_ms <= s_cap * capture_rate_hz even though 12 m/s and 0.5 Hz
// are each individually within their own bounds (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsSpeedCaptureRateInconsistency)
{
    auto request = commonSurveyRequest(refRect());
    request.survey_speed_ms = 12.0f;
    request.capture_rate_hz = 0.5f;
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("speed"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — degenerate (zero-area / collinear) polygon (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsZeroAreaPolygon)
{
    auto request = commonSurveyRequest(collinearPolygon());
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    // Rejected for degeneracy, not misclassified as self-intersecting/convex.
    EXPECT_NE(statuses_[0].detail.find("degenerate"), std::string::npos) << statuses_[0].detail;
}

// TS-02 — non-finite (NaN) numeric parameter (Verifies: MAP-6)
TEST_F(SurveyDispatchTest, RejectsNonFiniteAltitude)
{
    auto request = commonSurveyRequest(refRect());
    request.survey_altitude_m = std::numeric_limits<float>::quiet_NaN();
    publishAndWait(request);

    EXPECT_EQ(missions_.size(), 0u);
    ASSERT_EQ(statuses_.size(), 1u);
    EXPECT_EQ(statuses_[0].state, "rejected");
    EXPECT_NE(statuses_[0].detail.find("finite"), std::string::npos) << statuses_[0].detail;
}
