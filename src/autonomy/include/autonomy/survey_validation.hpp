#pragma once

// Pure validation logic for survey Mission specs — DES-003 "autonomy_node
// behavior", rule 2. No rclcpp::Node / ROS graph dependency (only the
// generated message structs), so it is unit-testable on its own, mirroring
// the navigation SurveyPlanner (DES-003 task T1.4).
//
// Implements: MAP-6

#include <string>

#include <drone_autonomy_msgs/msg/mission.hpp>
#include <geometry_msgs/msg/polygon.hpp>

namespace survey_validation
{

// DES-003 D4 camera model (OAK-D RGB). Mirrored here (not shared with
// navigation_node's declared parameters) only to compute the along-track
// capture spacing needed for the speed/capture-rate consistency rule below;
// the full footprint/lane generation lives in navigation's SurveyPlanner.
constexpr double kCameraVfovDeg = 55.0;

// Recorder default capture rate used when Mission.capture_rate_hz == 0.
constexpr double kRecorderDefaultCaptureRateHz = 2.0;

// DES-003 validation bounds.
constexpr float kMinAltitudeM = 10.0f;
constexpr float kMaxAltitudeM = 120.0f;
constexpr float kMinOverlap = 0.30f;
constexpr float kMaxOverlap = 0.95f;
constexpr float kMinCaptureRateHz = 0.5f;
constexpr float kMaxCaptureRateHz = 10.0f;
constexpr float kMinSpeedMs = 1.0f;
constexpr float kMaxSpeedMs = 12.0f;

struct ValidationResult
{
    bool ok{false};
    std::string detail;  // empty when ok; names the failed rule otherwise
};

// Segment-sweep self-intersection test: true if any two non-adjacent edges
// of `polygon` cross (or overlap collinearly). Polygons with fewer than 3
// vertices are reported as non-intersecting here — the vertex-count rule is
// checked separately by validate_survey_mission().
bool polygon_self_intersects(const geometry_msgs::msg::Polygon & polygon);

// Convexity test (DES-003 D5 — v1 accepts convex polygons only). For a simple
// (non-self-intersecting) polygon this is true iff the sign of the cross
// product of consecutive edge directions never changes; collinear turns
// (cross ~0 within kEps) are permitted. Polygons with fewer than 3 vertices
// are reported as non-convex. Meaningful only for simple polygons — callers
// run the self-intersection check first (validate_survey_mission does).
bool polygon_is_convex(const geometry_msgs::msg::Polygon & polygon);

// Runs every DES-003 "autonomy_node behavior" rule-2 check, in the order the
// design doc enumerates them, short-circuiting on the first failure so the
// returned detail names a single, unambiguous failed rule:
//   1. survey_polygon has >= 3 vertices
//   2. survey_polygon is not self-intersecting
//   3. survey_polygon is convex (DES-003 D5, v1)
//   4. 10.0 <= survey_altitude_m <= 120.0
//   5. forward_overlap and side_overlap in [0.30, 0.95]
//   6. capture_rate_hz is 0 or in [0.5, 10.0]
//   7. survey_speed_ms in [1.0, 12.0]
//   8. survey_speed_ms <= s_cap * effective_capture_rate_hz (s_cap derived
//      from survey_altitude_m, forward_overlap and kCameraVfovDeg; the
//      recorder default capture rate is used when capture_rate_hz == 0)
ValidationResult validate_survey_mission(const drone_autonomy_msgs::msg::Mission & mission);

}  // namespace survey_validation
