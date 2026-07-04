#pragma once

// Boustrophedon (lawnmower) coverage-trajectory generator for the "survey"
// mission type — DES-003 "navigation_node behavior".
//
// Pure geometry, no ROS dependencies: operates on plain 2D points so it is
// unit-testable in isolation (TP-002 TS-03) without a ROS graph. The owning
// navigation_node wraps this class to convert drone_autonomy_msgs::msg::
// Mission / Trajectory to and from these plain types.
//
// Implements: MAP-1

#include <cstddef>
#include <string>
#include <vector>

namespace navigation
{

// A 2D point (meters) in the mission's local ENU plan frame.
struct Point2D
{
    double x{0.0};
    double y{0.0};
};

// One generated survey waypoint.
struct SurveyWaypoint
{
    double x{0.0};
    double y{0.0};
    double z{0.0};    // AGL altitude, == survey_altitude_m for every waypoint
    double yaw{0.0};  // radians, aligned with this waypoint's lane direction of travel
    std::size_t lane_index{0};  // 0-based lane number in generation order (serpentine)
};

// Node parameters governing plan generation — DES-003 "navigation_node
// behavior", step 6. Defaults match the design doc exactly.
struct SurveyPlannerParams
{
    double camera_hfov_deg{69.0};
    double camera_vfov_deg{55.0};
    double max_lane_length_m{500.0};  // reject any generated lane longer than this
    double boundary_margin_m{0.0};    // extra inset from the polygon boundary, added to W/2
};

// Result of a single generate() call.
struct SurveyPlanResult
{
    bool success{false};
    std::string error;  // empty on success; names the failed rule otherwise

    std::vector<SurveyWaypoint> waypoints;  // empty on failure

    // Derived geometry, exposed for callers/tests (DES-003 steps 1-2).
    double footprint_width_m{0.0};    // W = 2*h*tan(HFOV/2)   (cross-track)
    double footprint_height_m{0.0};   // H = 2*h*tan(VFOV/2)   (along-track)
    double lane_spacing_m{0.0};       // s_lane = W*(1 - side_overlap)
    double capture_spacing_m{0.0};    // s_cap  = H*(1 - forward_overlap)
    double sweep_direction_rad{0.0};  // orientation of the longest polygon edge (lane 0 direction)
};

// Pure-geometry boustrophedon survey trajectory generator. See
// docs/design/DES-003-survey-mission-coverage-trajectory.md
// ("navigation_node behavior") for the algorithm this implements step by
// step; do not re-derive the algorithm from this comment alone.
class SurveyPlanner
{
public:
    explicit SurveyPlanner(const SurveyPlannerParams & params = SurveyPlannerParams());

    // Generates a coverage plan for a convex polygon.
    //
    // polygon:          vertices in order (CW or CCW), >= 3, z ignored by caller
    // altitude_m:       AGL altitude driving the footprint size (step 1)
    // forward_overlap:  fractional along-track photo overlap, [0, 1)
    // side_overlap:     fractional cross-track photo overlap, [0, 1)
    //
    // Returns success=false with a populated error and no waypoints for any
    // degenerate/non-convex/self-intersecting polygon, invalid overlap/
    // altitude input, or a generated lane longer than max_lane_length_m
    // (Safety impact, DES-003: reject, emit no trajectory).
    SurveyPlanResult generate(
        const std::vector<Point2D> & polygon,
        double altitude_m,
        double forward_overlap,
        double side_overlap) const;

private:
    SurveyPlannerParams params_;
};

}  // namespace navigation
