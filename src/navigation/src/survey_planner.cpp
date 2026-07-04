#include "navigation/survey_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
// General-purpose tolerance for geometric comparisons in this file (meters
// for lengths/spacings, radians for angle-adjacent comparisons where noted).
constexpr double kEps = 1e-9;

double degToRad(double deg) { return deg * kPi / 180.0; }

double dot(const Point2D & a, const Point2D & b) { return a.x * b.x + a.y * b.y; }

// z-component of the 2D cross product (b-a) x (c-b), used for turn direction.
double turnCross(const Point2D & a, const Point2D & b, const Point2D & c)
{
    return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}

double polygonArea(const std::vector<Point2D> & poly)
{
    double s = 0.0;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Point2D & a = poly[i];
        const Point2D & b = poly[(i + 1) % n];
        s += a.x * b.y - b.x * a.y;
    }
    return std::fabs(s) * 0.5;
}

// True if all turns (consecutive-triple cross products) share the same sign,
// treating near-zero (collinear) turns as neutral. A polygon that is all
// collinear (no nonzero turn found) is reported convex here; it is rejected
// separately by the degenerate-area check in validatePolygon().
bool isConvexTurnSequence(const std::vector<Point2D> & poly)
{
    const std::size_t n = poly.size();
    int sign = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double cross = turnCross(poly[i], poly[(i + 1) % n], poly[(i + 2) % n]);
        if (std::fabs(cross) < kEps) {
            continue;
        }
        const int s = (cross > 0.0) ? 1 : -1;
        if (sign == 0) {
            sign = s;
        } else if (s != sign) {
            return false;
        }
    }
    return true;
}

// Standard orientation-based segment intersection test (proper crossings and
// collinear overlaps both count as an intersection). Used defensively to
// re-check for self-intersection independent of the convexity turn test —
// DES-003 Safety impact: "degenerate polygon ... defensively re-checked in
// SurveyPlanner".
int orientation(const Point2D & p, const Point2D & q, const Point2D & r)
{
    const double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (std::fabs(val) < kEps) return 0;
    return (val > 0.0) ? 1 : 2;
}

bool onSegment(const Point2D & p, const Point2D & q, const Point2D & r)
{
    return q.x <= std::max(p.x, r.x) + kEps && q.x >= std::min(p.x, r.x) - kEps &&
           q.y <= std::max(p.y, r.y) + kEps && q.y >= std::min(p.y, r.y) - kEps;
}

bool segmentsIntersect(const Point2D & p1, const Point2D & p2, const Point2D & p3, const Point2D & p4)
{
    const int o1 = orientation(p1, p2, p3);
    const int o2 = orientation(p1, p2, p4);
    const int o3 = orientation(p3, p4, p1);
    const int o4 = orientation(p3, p4, p2);

    if (o1 != o2 && o3 != o4) {
        return true;
    }
    if (o1 == 0 && onSegment(p1, p3, p2)) return true;
    if (o2 == 0 && onSegment(p1, p4, p2)) return true;
    if (o3 == 0 && onSegment(p3, p1, p4)) return true;
    if (o4 == 0 && onSegment(p3, p2, p4)) return true;
    return false;
}

bool polygonSelfIntersects(const std::vector<Point2D> & poly)
{
    const std::size_t n = poly.size();
    if (n < 4) {
        return false;  // a triangle cannot self-intersect
    }
    for (std::size_t i = 0; i < n; ++i) {
        const Point2D & a1 = poly[i];
        const Point2D & a2 = poly[(i + 1) % n];
        // Only test non-adjacent edges; adjacent edges legitimately share a vertex.
        for (std::size_t j = i + 1; j < n; ++j) {
            if (j == i) continue;
            const bool adjacent = (j == i + 1) || (i == 0 && j == n - 1);
            if (adjacent) continue;
            const Point2D & b1 = poly[j];
            const Point2D & b2 = poly[(j + 1) % n];
            if (segmentsIntersect(a1, a2, b1, b2)) {
                return true;
            }
        }
    }
    return false;
}

// DES-003 Safety impact / D5: reject degenerate, non-convex, or
// self-intersecting polygons — returns false and fills `error` on any
// failure, true otherwise.
bool validatePolygon(const std::vector<Point2D> & poly, std::string & error)
{
    if (poly.size() < 3) {
        error = "survey polygon has fewer than 3 vertices";
        return false;
    }
    if (polygonArea(poly) < kEps) {
        error = "survey polygon is degenerate (zero area)";
        return false;
    }
    if (polygonSelfIntersects(poly)) {
        error = "survey polygon is self-intersecting";
        return false;
    }
    if (!isConvexTurnSequence(poly)) {
        error = "survey polygon is non-convex (v1 supports convex polygons only)";
        return false;
    }
    return true;
}

// Longest-edge direction (DES-003 step 3): unit vector along the polygon's
// longest edge. Ties keep the first edge encountered (deterministic).
Point2D longestEdgeDirection(const std::vector<Point2D> & poly)
{
    const std::size_t n = poly.size();
    double best_len2 = -1.0;
    Point2D best_dir{1.0, 0.0};
    for (std::size_t i = 0; i < n; ++i) {
        const Point2D & a = poly[i];
        const Point2D & b = poly[(i + 1) % n];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        if (len2 > best_len2) {
            best_len2 = len2;
            const double len = std::sqrt(len2);
            best_dir = Point2D{dx / len, dy / len};
        }
    }
    return best_dir;
}

// Generates positions from `start` to `end` (inclusive) stepped by `step`:
// step, step, ..., then a final (possibly shorter) segment landing exactly
// on `end`. If `end <= start` (within `kEps`, or `end < start` entirely —
// e.g. the footprint does not fit within the inset region twice over) a
// single midpoint position is returned instead, which is exactly the
// footprint-wider-than-extent centered fallback: the lane offsets are
// derived as (inset_low, inset_high) = (v_min + margin, v_max - margin), so
// their midpoint always equals (v_min + v_max) / 2 regardless of margin.
std::vector<double> stepPositions(double start, double end, double step)
{
    std::vector<double> out;
    if (end - start < kEps) {
        out.push_back((start + end) / 2.0);
        return out;
    }
    double v = start;
    while (v < end - kEps) {
        out.push_back(v);
        v += step;
    }
    out.push_back(end);
    // Guard against a near-duplicate final pair when `end` lands almost
    // exactly on the last regularly-stepped position.
    if (out.size() >= 2 && (out.back() - out[out.size() - 2]) < kEps) {
        out.erase(out.end() - 2);
    }
    return out;
}

}  // namespace

SurveyPlanner::SurveyPlanner(const SurveyPlannerParams & params) : params_(params) {}

// Implements: MAP-1
//
// Follows docs/design/DES-003-survey-mission-coverage-trajectory.md
// "navigation_node behavior" steps 1-4, 6 exactly (step 5 is the ROS
// publication wiring done by navigation_node, not this pure-geometry class).
SurveyPlanResult SurveyPlanner::generate(
    const std::vector<Point2D> & polygon,
    double altitude_m,
    double forward_overlap,
    double side_overlap) const
{
    SurveyPlanResult result;

    // --- Defensive input validation (Safety impact: degenerate polygon /
    // altitude out of bounds are rejected; re-checked here independent of
    // whatever upstream validation autonomy_node already performed). ---
    if (!validatePolygon(polygon, result.error)) {
        return result;
    }
    if (!std::isfinite(altitude_m) || altitude_m <= 0.0) {
        result.error = "survey_altitude_m must be a positive, finite number";
        return result;
    }
    if (!std::isfinite(forward_overlap) || forward_overlap < 0.0 || forward_overlap >= 1.0) {
        result.error = "forward_overlap must be in [0, 1)";
        return result;
    }
    if (!std::isfinite(side_overlap) || side_overlap < 0.0 || side_overlap >= 1.0) {
        result.error = "side_overlap must be in [0, 1)";
        return result;
    }

    // --- Step 1: footprint at altitude h. ---
    const double half_hfov = degToRad(params_.camera_hfov_deg) / 2.0;
    const double half_vfov = degToRad(params_.camera_vfov_deg) / 2.0;
    const double W = 2.0 * altitude_m * std::tan(half_hfov);
    const double H = 2.0 * altitude_m * std::tan(half_vfov);

    if (!(W > kEps) || !(H > kEps)) {
        result.error = "computed footprint is non-positive (check altitude/FOV parameters)";
        return result;
    }

    // --- Step 2: lane spacing and capture spacing. ---
    const double s_lane = W * (1.0 - side_overlap);
    const double s_cap = H * (1.0 - forward_overlap);
    if (!(s_lane > kEps) || !(s_cap > kEps)) {
        result.error = "computed lane/capture spacing is non-positive (check overlap parameters)";
        return result;
    }

    // --- Step 3: sweep direction = longest-edge orientation; lanes are the
    // polygon's intersections with lines offset by s_lane perpendicular to
    // that direction, inset by W/2 (+ boundary_margin_m) from the boundary.
    const Point2D u_hat = longestEdgeDirection(polygon);
    const Point2D v_hat{-u_hat.y, u_hat.x};  // perpendicular, 90 deg CCW rotation

    double v_min = std::numeric_limits<double>::infinity();
    double v_max = -std::numeric_limits<double>::infinity();
    std::vector<double> u_coords(polygon.size());
    std::vector<double> v_coords(polygon.size());
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        u_coords[i] = dot(polygon[i], u_hat);
        v_coords[i] = dot(polygon[i], v_hat);
        v_min = std::min(v_min, v_coords[i]);
        v_max = std::max(v_max, v_coords[i]);
    }

    const double effective_margin = W / 2.0 + std::max(0.0, params_.boundary_margin_m);
    const double lane_v_start = v_min + effective_margin;
    const double lane_v_end = v_max - effective_margin;
    const std::vector<double> lane_offsets = stepPositions(lane_v_start, lane_v_end, s_lane);

    // --- Steps 3-4: per lane, intersect the polygon with the offset line to
    // get the lane chord, then place waypoints every s_cap along it
    // (serpentine direction), yaw aligned with the lane, z = altitude. ---
    result.waypoints.reserve(lane_offsets.size() * 8);

    for (std::size_t lane_i = 0; lane_i < lane_offsets.size(); ++lane_i) {
        const double offset = lane_offsets[lane_i];

        std::vector<double> intersections;
        const std::size_t n = polygon.size();
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            const double va = v_coords[i];
            const double vb = v_coords[j];
            const double ua = u_coords[i];
            const double ub = u_coords[j];
            if (std::fabs(vb - va) < kEps) {
                // Edge runs parallel to the sweep direction (constant v).
                if (std::fabs(va - offset) < kEps) {
                    intersections.push_back(ua);
                    intersections.push_back(ub);
                }
                continue;
            }
            const double vlo = std::min(va, vb);
            const double vhi = std::max(va, vb);
            if (offset >= vlo - kEps && offset <= vhi + kEps) {
                double t = (offset - va) / (vb - va);
                t = std::clamp(t, 0.0, 1.0);
                intersections.push_back(ua + t * (ub - ua));
            }
        }

        if (intersections.size() < 2) {
            // Degenerate touch (line only grazes a single vertex) — no
            // usable chord at this offset; skip defensively rather than
            // emit a spurious single-point "lane".
            continue;
        }
        const double u_start = *std::min_element(intersections.begin(), intersections.end());
        const double u_end = *std::max_element(intersections.begin(), intersections.end());
        const double chord_length = u_end - u_start;

        if (chord_length > params_.max_lane_length_m) {
            result.error = "generated lane length exceeds max_lane_length_m";
            result.waypoints.clear();
            return result;
        }

        // Serpentine: even lanes travel +u (u_start -> u_end), odd lanes -u.
        const bool forward_dir = (lane_i % 2 == 0);
        const double u_from = forward_dir ? u_start : u_end;
        const double dir_sign = forward_dir ? 1.0 : -1.0;
        const Point2D travel_dir = forward_dir ? u_hat : Point2D{-u_hat.x, -u_hat.y};
        const double yaw = std::atan2(travel_dir.y, travel_dir.x);

        const std::vector<double> step_ts = stepPositions(0.0, chord_length, s_cap);
        for (const double t : step_ts) {
            const double u_coord = u_from + dir_sign * t;
            SurveyWaypoint wp;
            wp.x = u_coord * u_hat.x + offset * v_hat.x;
            wp.y = u_coord * u_hat.y + offset * v_hat.y;
            wp.z = altitude_m;
            wp.yaw = yaw;
            wp.lane_index = lane_i;
            result.waypoints.push_back(wp);
        }
    }

    if (result.waypoints.empty()) {
        result.error = "no waypoints generated for the given polygon/parameters";
        return result;
    }

    result.success = true;
    result.footprint_width_m = W;
    result.footprint_height_m = H;
    result.lane_spacing_m = s_lane;
    result.capture_spacing_m = s_cap;
    result.sweep_direction_rad = std::atan2(u_hat.y, u_hat.x);
    return result;
}

}  // namespace navigation
