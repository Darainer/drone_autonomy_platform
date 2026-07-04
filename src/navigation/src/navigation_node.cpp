#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <drone_autonomy_msgs/msg/mission.hpp>
#include <drone_autonomy_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "navigation/survey_planner.hpp"

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node")
    {
        // DES-003 "navigation_node behavior", step 6: declared parameters,
        // defaults per the design doc.
        this->declare_parameter<double>("camera_hfov_deg", 69.0);
        this->declare_parameter<double>("camera_vfov_deg", 55.0);
        this->declare_parameter<double>("max_lane_length_m", 500.0);
        this->declare_parameter<double>("boundary_margin_m", 0.0);

        navigation::SurveyPlannerParams planner_params;
        planner_params.camera_hfov_deg = this->get_parameter("camera_hfov_deg").as_double();
        planner_params.camera_vfov_deg = this->get_parameter("camera_vfov_deg").as_double();
        planner_params.max_lane_length_m = this->get_parameter("max_lane_length_m").as_double();
        planner_params.boundary_margin_m = this->get_parameter("boundary_margin_m").as_double();
        survey_planner_ = std::make_unique<navigation::SurveyPlanner>(planner_params);

        mission_subscriber_ = this->create_subscription<drone_autonomy_msgs::msg::Mission>(
            "~/mission", 10, std::bind(&NavigationNode::missionCallback, this, std::placeholders::_1));

        trajectory_publisher_ = this->create_publisher<drone_autonomy_msgs::msg::Trajectory>("~/trajectory", 10);
    }

private:
    void missionCallback(const drone_autonomy_msgs::msg::Mission::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received mission: %s", msg->mission_id.c_str());

        // DES-003 "navigation_node behavior", step 5: generate + publish a
        // Trajectory only for mission_type == "survey". Non-survey missions
        // keep current behavior (log only, no publication).
        if (msg->mission_type != "survey") {
            return;
        }

        handleSurveyMission(*msg);
    }

    void handleSurveyMission(const drone_autonomy_msgs::msg::Mission & mission)
    {
        std::vector<navigation::Point2D> polygon;
        polygon.reserve(mission.survey_polygon.points.size());
        for (const auto & pt : mission.survey_polygon.points) {
            polygon.push_back(navigation::Point2D{static_cast<double>(pt.x), static_cast<double>(pt.y)});
        }

        const navigation::SurveyPlanResult result = survey_planner_->generate(
            polygon,
            static_cast<double>(mission.survey_altitude_m),
            static_cast<double>(mission.forward_overlap),
            static_cast<double>(mission.side_overlap));

        if (!result.success) {
            // Safety impact (DES-003): degenerate/non-convex polygon or an
            // over-length lane is rejected — emit no trajectory.
            RCLCPP_ERROR(this->get_logger(), "survey mission %s: trajectory generation failed: %s",
                         mission.mission_id.c_str(), result.error.c_str());
            return;
        }

        auto trajectory = drone_autonomy_msgs::msg::Trajectory();
        trajectory.header.stamp = this->now();
        trajectory.header.frame_id = mission.header.frame_id;
        trajectory.path.header = trajectory.header;

        trajectory.path.poses.reserve(result.waypoints.size());
        for (const auto & wp : result.waypoints) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = trajectory.header;
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = wp.z;

            // Yaw-only orientation (rotation about Z): no roll/pitch for a
            // planar survey sweep.
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
            pose.pose.orientation.w = std::cos(wp.yaw / 2.0);

            trajectory.path.poses.push_back(pose);
        }

        trajectory_publisher_->publish(trajectory);
        RCLCPP_INFO(this->get_logger(), "survey mission %s: published trajectory with %zu waypoints",
                    mission.mission_id.c_str(), result.waypoints.size());
    }

    rclcpp::Subscription<drone_autonomy_msgs::msg::Mission>::SharedPtr mission_subscriber_;
    rclcpp::Publisher<drone_autonomy_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;
    std::unique_ptr<navigation::SurveyPlanner> survey_planner_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}
