// TP-002 TS-03 — Coverage geometry properties (Verifies: MAP-1)
//
// Exercises navigation::SurveyPlanner (pure geometry, no ROS graph) against
// the two TP-002 F1 reference fixtures (REF-RECT, REF-HEX) at the common
// parameters and at the named parameter extremes, and checks every TS-03
// pass assertion:
//   - adjacent lane spacing <= (1 - side_overlap) * W + 1 cm
//   - along-track waypoint spacing <= (1 - forward_overlap) * H + 1 cm
//   - every waypoint within the polygon dilated by W/2
//   - union of per-waypoint footprints covers >= 99% of the polygon area
//   - every waypoint z == survey_altitude_m
//   - yaw parallel to lane direction, +/- 1 degree
//
// Verifies: MAP-1

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "navigation/survey_planner.hpp"

namespace
{

using navigation::Point2D;
using navigation::SurveyPlanner;
using navigation::SurveyPlannerParams;
using navigation::SurveyPlanResult;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// ---------------------------------------------------------------------
// F1 reference fixtures (TP-002 "Reference fixtures").
// ---------------------------------------------------------------------

// REF-RECT: 100 m x 80 m rectangle.
std::vector<Point2D> refRect()
{
    return {{0.0, 0.0}, {100.0, 0.0}, {100.0, 80.0}, {0.0, 80.0}};
}

// REF-HEX: convex 6-vertex polygon, ~0.79 ha (7875 sqm), with one clearly
// longest edge (95.0 m, distinct from all others) oriented ~25 degrees off
// the x-axis so the test genuinely exercises the rotated-frame math (not
// just the axis-aligned case already covered by REF-RECT).
std::vector<Point2D> refHex()
{
    return {
        {187.321, 227.189},
        {240.784, 219.018},
        {326.883, 259.167},
        {345.925, 301.147},
        {292.463, 309.319},
        {219.958, 275.509},
    };
}

// Signed distance from `p` to the boundary of convex polygon `poly`:
// <= 0 (or 0) when inside, positive distance-to-nearest-edge when outside.
// Used for the "waypoint within polygon dilated by W/2" assertion, which is
// equivalent to distanceToPolygon(p) <= W/2 (+ tolerance).
double distanceToPolygon(const Point2D & p, const std::vector<Point2D> & poly)
{
    const std::size_t n = poly.size();

    // Point-in-convex-polygon: same-sign cross product against every edge
    // (poly may be wound CW or CCW; check consistency against the first
    // nonzero edge sign rather than assuming a winding order).
    bool inside = true;
    int ref_sign = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto & a = poly[i];
        const auto & b = poly[(i + 1) % n];
        const double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if (std::fabs(cross) < 1e-9) continue;
        const int s = (cross > 0.0) ? 1 : -1;
        if (ref_sign == 0) {
            ref_sign = s;
        } else if (s != ref_sign) {
            inside = false;
            break;
        }
    }
    if (inside) return 0.0;

    // Outside: minimum distance to any edge segment.
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
        const auto & a = poly[i];
        const auto & b = poly[(i + 1) % n];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = len2 > 1e-12 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        const double cx = a.x + t * dx;
        const double cy = a.y + t * dy;
        const double d = std::hypot(p.x - cx, p.y - cy);
        best = std::min(best, d);
    }
    return best;
}

// Grid-rasterization coverage/containment checker: samples a fine grid over
// the polygon's bounding box, and for cells whose center lies inside the
// polygon, checks whether that center is covered by the rectangular
// footprint (W across-track x H along-track, centered at the waypoint,
// oriented along its yaw) of at least one waypoint. Returns the fraction of
// inside-polygon grid cells that are covered (an area-ratio estimate).
double coverageFraction(
    const std::vector<Point2D> & poly,
    const std::vector<navigation::SurveyWaypoint> & waypoints,
    double W, double H)
{
    double minx = poly[0].x, maxx = poly[0].x, miny = poly[0].y, maxy = poly[0].y;
    for (const auto & v : poly) {
        minx = std::min(minx, v.x);
        maxx = std::max(maxx, v.x);
        miny = std::min(miny, v.y);
        maxy = std::max(maxy, v.y);
    }
    const double bbox_w = maxx - minx;
    const double bbox_h = maxy - miny;
    double cell = std::max(0.05, std::min(bbox_w, bbox_h) / 300.0);
    const std::size_t nx = static_cast<std::size_t>(std::ceil(bbox_w / cell)) + 1;
    const std::size_t ny = static_cast<std::size_t>(std::ceil(bbox_h / cell)) + 1;

    std::vector<uint8_t> inside_mask(nx * ny, 0);
    std::vector<uint8_t> covered_mask(nx * ny, 0);

    auto idx = [&](std::size_t i, std::size_t j) { return i * ny + j; };

    std::size_t inside_count = 0;
    for (std::size_t i = 0; i < nx; ++i) {
        for (std::size_t j = 0; j < ny; ++j) {
            const Point2D c{minx + (i + 0.5) * cell, miny + (j + 0.5) * cell};
            if (distanceToPolygon(c, poly) < 1e-9) {
                inside_mask[idx(i, j)] = 1;
                ++inside_count;
            }
        }
    }
    if (inside_count == 0) return 0.0;

    const double tol = 0.01;  // 1 cm, matching the spacing tolerances elsewhere
    for (const auto & wp : waypoints) {
        const double fx = std::cos(wp.yaw);
        const double fy = std::sin(wp.yaw);
        const double rx = -fy;
        const double ry = fx;
        const double halfH = H / 2.0 + tol;
        const double halfW = W / 2.0 + tol;

        // AABB of the rotated footprint rectangle, to bound the cell scan.
        const double dx = std::fabs(fx) * halfH + std::fabs(rx) * halfW;
        const double dy = std::fabs(fy) * halfH + std::fabs(ry) * halfW;

        const double bx0 = std::max(minx, wp.x - dx);
        const double bx1 = std::min(maxx, wp.x + dx);
        const double by0 = std::max(miny, wp.y - dy);
        const double by1 = std::min(maxy, wp.y + dy);
        if (bx1 < bx0 || by1 < by0) continue;

        const auto i0 = static_cast<std::size_t>(std::max(0.0, (bx0 - minx) / cell));
        const auto i1 = static_cast<std::size_t>(std::min<double>(nx - 1, (bx1 - minx) / cell + 1));
        const auto j0 = static_cast<std::size_t>(std::max(0.0, (by0 - miny) / cell));
        const auto j1 = static_cast<std::size_t>(std::min<double>(ny - 1, (by1 - miny) / cell + 1));

        for (std::size_t i = i0; i <= i1 && i < nx; ++i) {
            for (std::size_t j = j0; j <= j1 && j < ny; ++j) {
                if (!inside_mask[idx(i, j)] || covered_mask[idx(i, j)]) continue;
                const double cx = minx + (i + 0.5) * cell;
                const double cy = miny + (j + 0.5) * cell;
                const double along = (cx - wp.x) * fx + (cy - wp.y) * fy;
                const double across = (cx - wp.x) * rx + (cy - wp.y) * ry;
                if (std::fabs(along) <= halfH && std::fabs(across) <= halfW) {
                    covered_mask[idx(i, j)] = 1;
                }
            }
        }
    }

    std::size_t covered_count = 0;
    for (std::size_t k = 0; k < inside_mask.size(); ++k) {
        if (inside_mask[k] && covered_mask[k]) ++covered_count;
    }
    return static_cast<double>(covered_count) / static_cast<double>(inside_count);
}

double angleDiffDeg(double yaw_rad, double target_rad)
{
    double d = (yaw_rad - target_rad) * 180.0 / kPi;
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return std::fabs(d);
}

struct SurveyParams
{
    std::string label;
    double altitude_m;
    double forward_overlap;
    double side_overlap;
};

// Common parameters (TP-002 "Reference fixtures") plus the named parameter
// extremes: altitude 10/120 m, overlaps 0.30/0.95 (applied to both
// forward_overlap and side_overlap, per DES-003's shared [0.30, 0.95] valid
// range for both fields).
std::vector<SurveyParams> parameterSets()
{
    return {
        {"common", 40.0, 0.75, 0.60},
        {"alt_min_10m", 10.0, 0.75, 0.60},
        {"alt_max_120m", 120.0, 0.75, 0.60},
        {"overlap_min_0_30", 40.0, 0.30, 0.30},
        {"overlap_max_0_95", 40.0, 0.95, 0.95},
    };
}

// Runs every TS-03 assertion for one (polygon, params) case.
void checkCoverageGeometryProperties(
    const std::string & fixture_name,
    const std::vector<Point2D> & polygon,
    const SurveyParams & p)
{
    SCOPED_TRACE("fixture=" + fixture_name + " params=" + p.label);

    SurveyPlannerParams params;  // defaults: HFOV 69, VFOV 55, per DES-003 D4
    SurveyPlanner planner(params);

    const SurveyPlanResult result =
        planner.generate(polygon, p.altitude_m, p.forward_overlap, p.side_overlap);

    ASSERT_TRUE(result.success) << "generate() failed: " << result.error;
    ASSERT_FALSE(result.waypoints.empty());

    // Independent recomputation of W/H/s_lane/s_cap (DES-003 steps 1-2),
    // decoupled from the planner's own self-reported values, so the bound
    // assertions below test the planner's actual output, not just its
    // internal self-consistency.
    const double half_hfov = (params.camera_hfov_deg / 2.0) * kDegToRad;
    const double half_vfov = (params.camera_vfov_deg / 2.0) * kDegToRad;
    const double W = 2.0 * p.altitude_m * std::tan(half_hfov);
    const double H = 2.0 * p.altitude_m * std::tan(half_vfov);
    const double s_lane_max = (1.0 - p.side_overlap) * W;
    const double s_cap_max = (1.0 - p.forward_overlap) * H;

    EXPECT_NEAR(result.footprint_width_m, W, 1e-6);
    EXPECT_NEAR(result.footprint_height_m, H, 1e-6);

    constexpr double kLenTol = 0.01;   // 1 cm, per TS-03
    constexpr double kAngTolDeg = 1.0;  // +/- 1 degree, per TS-03

    // --- z == survey_altitude_m, yaw parallel to lane direction +/- 1 deg. ---
    for (const auto & wp : result.waypoints) {
        EXPECT_DOUBLE_EQ(wp.z, p.altitude_m);
        const double diff_fwd = angleDiffDeg(wp.yaw, result.sweep_direction_rad);
        const double diff_rev = angleDiffDeg(wp.yaw, result.sweep_direction_rad + kPi);
        EXPECT_LE(std::min(diff_fwd, diff_rev), kAngTolDeg)
            << "yaw=" << wp.yaw << " sweep=" << result.sweep_direction_rad;
    }

    // --- every waypoint within polygon dilated by W/2. ---
    for (const auto & wp : result.waypoints) {
        const double d = distanceToPolygon(Point2D{wp.x, wp.y}, polygon);
        EXPECT_LE(d, W / 2.0 + kLenTol)
            << "waypoint (" << wp.x << "," << wp.y << ") distance " << d
            << " exceeds W/2=" << (W / 2.0);
    }

    // --- group by lane_index for the spacing assertions. ---
    std::map<std::size_t, std::vector<navigation::SurveyWaypoint>> lanes;
    for (const auto & wp : result.waypoints) {
        lanes[wp.lane_index].push_back(wp);
    }

    // --- adjacent lane spacing <= s_lane_max + 1cm: project one waypoint
    // per lane onto the perpendicular (v) direction and diff consecutive
    // lanes (lane_index is assigned in monotonically increasing v order by
    // construction). ---
    const double v_hat_x = -std::sin(result.sweep_direction_rad);
    const double v_hat_y = std::cos(result.sweep_direction_rad);
    std::vector<double> lane_v;
    lane_v.reserve(lanes.size());
    for (auto & [idx, wps] : lanes) {
        const auto & wp0 = wps.front();
        lane_v.push_back(wp0.x * v_hat_x + wp0.y * v_hat_y);
    }
    for (std::size_t i = 1; i < lane_v.size(); ++i) {
        const double spacing = std::fabs(lane_v[i] - lane_v[i - 1]);
        EXPECT_LE(spacing, s_lane_max + kLenTol)
            << "lane " << i - 1 << "->" << i << " spacing " << spacing;
    }

    // --- along-track waypoint spacing <= s_cap_max + 1cm, within each lane. ---
    for (auto & [idx, wps] : lanes) {
        for (std::size_t i = 1; i < wps.size(); ++i) {
            const double dx = wps[i].x - wps[i - 1].x;
            const double dy = wps[i].y - wps[i - 1].y;
            const double spacing = std::hypot(dx, dy);
            EXPECT_LE(spacing, s_cap_max + kLenTol)
                << "lane " << idx << " waypoint " << i - 1 << "->" << i << " spacing " << spacing;
        }
    }

    // --- union of per-waypoint footprints covers >= 99% of polygon area. ---
    const double coverage = coverageFraction(polygon, result.waypoints, W, H);
    EXPECT_GE(coverage, 0.99) << "coverage fraction " << coverage;
}

}  // namespace

// --- REF-RECT cases -----------------------------------------------------

TEST(SurveyPlannerTs03, RefRectCommon)
{
    // TS-03: common parameters (alt 40 m, overlaps 0.75/0.60) on REF-RECT.
    checkCoverageGeometryProperties("REF-RECT", refRect(), parameterSets()[0]);
}

TEST(SurveyPlannerTs03, RefRectAltMin10m)
{
    // TS-03: altitude extreme 10 m on REF-RECT.
    checkCoverageGeometryProperties("REF-RECT", refRect(), parameterSets()[1]);
}

TEST(SurveyPlannerTs03, RefRectAltMax120m)
{
    // TS-03: altitude extreme 120 m on REF-RECT (footprint W exceeds the
    // polygon's cross-sweep extent; exercises the single-centered-lane
    // fallback).
    checkCoverageGeometryProperties("REF-RECT", refRect(), parameterSets()[2]);
}

TEST(SurveyPlannerTs03, RefRectOverlapMin0_30)
{
    // TS-03: overlap extreme 0.30 (both forward/side) on REF-RECT.
    checkCoverageGeometryProperties("REF-RECT", refRect(), parameterSets()[3]);
}

TEST(SurveyPlannerTs03, RefRectOverlapMax0_95)
{
    // TS-03: overlap extreme 0.95 (both forward/side) on REF-RECT.
    checkCoverageGeometryProperties("REF-RECT", refRect(), parameterSets()[4]);
}

// --- REF-HEX cases -------------------------------------------------------

TEST(SurveyPlannerTs03, RefHexCommon)
{
    // TS-03: common parameters (alt 40 m, overlaps 0.75/0.60) on REF-HEX.
    checkCoverageGeometryProperties("REF-HEX", refHex(), parameterSets()[0]);
}

TEST(SurveyPlannerTs03, RefHexAltMin10m)
{
    // TS-03: altitude extreme 10 m on REF-HEX.
    checkCoverageGeometryProperties("REF-HEX", refHex(), parameterSets()[1]);
}

TEST(SurveyPlannerTs03, RefHexAltMax120m)
{
    // TS-03: altitude extreme 120 m on REF-HEX (single-lane fallback).
    checkCoverageGeometryProperties("REF-HEX", refHex(), parameterSets()[2]);
}

TEST(SurveyPlannerTs03, RefHexOverlapMin0_30)
{
    // TS-03: overlap extreme 0.30 (both forward/side) on REF-HEX.
    checkCoverageGeometryProperties("REF-HEX", refHex(), parameterSets()[3]);
}

TEST(SurveyPlannerTs03, RefHexOverlapMax0_95)
{
    // TS-03: overlap extreme 0.95 (both forward/side) on REF-HEX.
    checkCoverageGeometryProperties("REF-HEX", refHex(), parameterSets()[4]);
}

// --- Safety impact / D5: degenerate & non-convex rejection ---------------

TEST(SurveyPlannerTs03, RejectsTooFewVertices)
{
    // TS-03 / Safety impact: degenerate polygon -> error, no trajectory.
    navigation::SurveyPlanner planner;
    const std::vector<Point2D> two_point_poly = {{0.0, 0.0}, {10.0, 0.0}};
    const auto result = planner.generate(two_point_poly, 40.0, 0.75, 0.60);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.waypoints.empty());
    EXPECT_FALSE(result.error.empty());
}

TEST(SurveyPlannerTs03, RejectsSelfIntersectingBowtie)
{
    // TS-03 / Safety impact / D5: self-intersecting (bowtie) polygon ->
    // error, no trajectory.
    navigation::SurveyPlanner planner;
    const std::vector<Point2D> bowtie = {{0.0, 0.0}, {10.0, 10.0}, {10.0, 0.0}, {0.0, 10.0}};
    const auto result = planner.generate(bowtie, 40.0, 0.75, 0.60);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.waypoints.empty());
    EXPECT_FALSE(result.error.empty());
}

TEST(SurveyPlannerTs03, RejectsNonConvexPolygon)
{
    // TS-03 / Safety impact / D5: non-convex (concave "arrow") polygon ->
    // error, no trajectory (v1 supports convex polygons only).
    navigation::SurveyPlanner planner;
    const std::vector<Point2D> concave = {
        {0.0, 0.0}, {50.0, 0.0}, {50.0, 50.0}, {25.0, 20.0}, {0.0, 50.0}};
    const auto result = planner.generate(concave, 40.0, 0.75, 0.60);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.waypoints.empty());
    EXPECT_FALSE(result.error.empty());
}

TEST(SurveyPlannerTs03, RejectsLaneLongerThanMaxLaneLength)
{
    // TS-03 / Safety impact: a lane longer than max_lane_length_m -> error,
    // no trajectory. A 1000 m x 10 m rectangle at default max_lane_length_m
    // (500 m) forces a > 500 m lane.
    navigation::SurveyPlanner planner;
    const std::vector<Point2D> long_rect = {
        {0.0, 0.0}, {1000.0, 0.0}, {1000.0, 10.0}, {0.0, 10.0}};
    const auto result = planner.generate(long_rect, 40.0, 0.75, 0.60);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.waypoints.empty());
    EXPECT_FALSE(result.error.empty());
}

// No main() here: ament_add_gtest links gtest_main, which provides it. The
// standalone (non-ROS) verification build links -lgtest_main for the same
// reason — see the scratch CMakeLists used for local TS-03 verification.
