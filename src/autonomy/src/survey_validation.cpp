#include "autonomy/survey_validation.hpp"

#include <algorithm>
#include <cmath>

namespace survey_validation
{

namespace
{

using Point32 = geometry_msgs::msg::Point32;

constexpr double kEps = 1e-9;

// Below this absolute area (m^2) a polygon is treated as degenerate/zero-area
// (e.g. all vertices collinear): it has no interior for the coverage planner
// to sweep, so it must be rejected before dispatch rather than dispatched and
// silently producing no trajectory.
constexpr double kMinPolygonAreaM2 = 1e-3;

double cross(const Point32 & o, const Point32 & a, const Point32 & b)
{
    return (static_cast<double>(a.x) - o.x) * (static_cast<double>(b.y) - o.y) -
           (static_cast<double>(a.y) - o.y) * (static_cast<double>(b.x) - o.x);
}

// 1 = CCW, 2 = CW, 0 = collinear (within kEps).
int orientation(const Point32 & p, const Point32 & q, const Point32 & r)
{
    const double val = cross(p, q, r);
    if (val > kEps) {
        return 1;
    }
    if (val < -kEps) {
        return 2;
    }
    return 0;
}

// Precondition: p, q, r are collinear. True if q lies on segment [p, r].
bool on_segment(const Point32 & p, const Point32 & q, const Point32 & r)
{
    return std::min(p.x, r.x) - kEps <= q.x && q.x <= std::max(p.x, r.x) + kEps &&
           std::min(p.y, r.y) - kEps <= q.y && q.y <= std::max(p.y, r.y) + kEps;
}

bool segments_intersect(const Point32 & p1, const Point32 & q1, const Point32 & p2, const Point32 & q2)
{
    const int o1 = orientation(p1, q1, p2);
    const int o2 = orientation(p1, q1, q2);
    const int o3 = orientation(p2, q2, p1);
    const int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) {
        return true;
    }

    // Collinear special cases (an endpoint lies on the other segment).
    if (o1 == 0 && on_segment(p1, p2, q1)) {
        return true;
    }
    if (o2 == 0 && on_segment(p1, q2, q1)) {
        return true;
    }
    if (o3 == 0 && on_segment(p2, p1, q2)) {
        return true;
    }
    if (o4 == 0 && on_segment(p2, q1, q2)) {
        return true;
    }

    return false;
}

// True if every vertex coordinate is a finite number. NaN/Inf vertices would
// make the cross-product and area math below silently non-finite and let a
// malformed polygon pass, so they are rejected up front.
bool polygon_all_finite(const geometry_msgs::msg::Polygon & polygon)
{
    for (const auto & p : polygon.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            return false;
        }
    }
    return true;
}

// Absolute polygon area via the shoelace formula. Precondition: vertices are
// finite (checked separately). Zero (within kMinPolygonAreaM2) means the
// vertices are collinear / the polygon is degenerate.
double polygon_area(const geometry_msgs::msg::Polygon & polygon)
{
    const auto & pts = polygon.points;
    const std::size_t n = pts.size();
    double twice_area = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Point32 & a = pts[i];
        const Point32 & b = pts[(i + 1) % n];
        twice_area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    return std::abs(twice_area) * 0.5;
}

}  // namespace

bool polygon_self_intersects(const geometry_msgs::msg::Polygon & polygon)
{
    const auto & pts = polygon.points;
    const std::size_t n = pts.size();
    if (n < 3) {
        return false;
    }

    for (std::size_t i = 0; i < n; ++i) {
        const Point32 & a1 = pts[i];
        const Point32 & a2 = pts[(i + 1) % n];
        for (std::size_t j = i + 1; j < n; ++j) {
            const bool adjacent = (j == i + 1) || (i == 0 && j == n - 1);
            if (adjacent) {
                continue;
            }
            const Point32 & b1 = pts[j];
            const Point32 & b2 = pts[(j + 1) % n];
            if (segments_intersect(a1, a2, b1, b2)) {
                return true;
            }
        }
    }
    return false;
}

bool polygon_is_convex(const geometry_msgs::msg::Polygon & polygon)
{
    const auto & pts = polygon.points;
    const std::size_t n = pts.size();
    if (n < 3) {
        return false;
    }

    // Walk the vertices, taking the cross product of consecutive edge
    // directions (the turn at each vertex). For a convex simple polygon every
    // non-zero turn has the same sign; collinear turns (|cross| <= kEps) are
    // permitted and skipped. A sign flip means a reflex vertex → concave.
    int sign = 0;  // 0 = no non-collinear turn seen yet
    for (std::size_t i = 0; i < n; ++i) {
        const Point32 & a = pts[i];
        const Point32 & b = pts[(i + 1) % n];
        const Point32 & c = pts[(i + 2) % n];
        const double z = cross(a, b, c);
        if (z > kEps) {
            if (sign < 0) {
                return false;
            }
            sign = 1;
        } else if (z < -kEps) {
            if (sign > 0) {
                return false;
            }
            sign = -1;
        }
    }
    return true;
}

ValidationResult validate_survey_mission(const drone_autonomy_msgs::msg::Mission & mission)
{
    const auto & polygon = mission.survey_polygon;
    const std::size_t n = polygon.points.size();

    if (n < 3) {
        return {false, "survey_polygon must have at least 3 vertices (got " +
                           std::to_string(n) + ")"};
    }

    if (!polygon_all_finite(polygon)) {
        return {false, "survey_polygon has a non-finite (NaN/Inf) vertex"};
    }

    if (polygon_self_intersects(polygon)) {
        return {false, "survey_polygon is self-intersecting"};
    }

    if (polygon_area(polygon) < kMinPolygonAreaM2) {
        return {false, "survey_polygon is degenerate (near-zero area / collinear vertices)"};
    }

    if (!polygon_is_convex(polygon)) {
        return {false, "survey_polygon must be convex (v1, DES-003 D5)"};
    }

    if (!std::isfinite(mission.survey_altitude_m) || !std::isfinite(mission.forward_overlap) ||
        !std::isfinite(mission.side_overlap) || !std::isfinite(mission.capture_rate_hz) ||
        !std::isfinite(mission.survey_speed_ms)) {
        return {false, "survey mission parameters must be finite numbers "
                       "(altitude, overlaps, speed, capture_rate)"};
    }

    if (mission.survey_altitude_m < kMinAltitudeM || mission.survey_altitude_m > kMaxAltitudeM) {
        return {false, "survey_altitude_m out of range [10.0, 120.0] (got " +
                           std::to_string(mission.survey_altitude_m) + ")"};
    }

    if (mission.forward_overlap < kMinOverlap || mission.forward_overlap > kMaxOverlap) {
        return {false, "forward_overlap out of range [0.30, 0.95] (got " +
                           std::to_string(mission.forward_overlap) + ")"};
    }

    if (mission.side_overlap < kMinOverlap || mission.side_overlap > kMaxOverlap) {
        return {false, "side_overlap out of range [0.30, 0.95] (got " +
                           std::to_string(mission.side_overlap) + ")"};
    }

    if (mission.capture_rate_hz != 0.0f &&
        (mission.capture_rate_hz < kMinCaptureRateHz || mission.capture_rate_hz > kMaxCaptureRateHz)) {
        return {false, "capture_rate_hz must be 0 or in range [0.5, 10.0] (got " +
                           std::to_string(mission.capture_rate_hz) + ")"};
    }

    if (mission.survey_speed_ms < kMinSpeedMs || mission.survey_speed_ms > kMaxSpeedMs) {
        return {false, "survey_speed_ms out of range [1.0, 12.0] (got " +
                           std::to_string(mission.survey_speed_ms) + ")"};
    }

    const double effective_rate_hz = (mission.capture_rate_hz == 0.0f)
                                          ? kRecorderDefaultCaptureRateHz
                                          : static_cast<double>(mission.capture_rate_hz);
    const double vfov_rad = kCameraVfovDeg * M_PI / 180.0;
    const double footprint_h_m = 2.0 * mission.survey_altitude_m * std::tan(vfov_rad / 2.0);
    const double s_cap_m = footprint_h_m * (1.0 - mission.forward_overlap);
    const double max_consistent_speed_ms = s_cap_m * effective_rate_hz;

    if (mission.survey_speed_ms > max_consistent_speed_ms) {
        return {false, "survey_speed_ms (" + std::to_string(mission.survey_speed_ms) +
                           ") exceeds the capture-rate-consistent max (" +
                           std::to_string(max_consistent_speed_ms) + ") at capture_rate_hz=" +
                           std::to_string(effective_rate_hz)};
    }

    return {true, ""};
}

}  // namespace survey_validation
