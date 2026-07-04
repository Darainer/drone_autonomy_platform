#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <drone_autonomy_msgs/msg/mission.hpp>
#include <drone_autonomy_msgs/msg/mission_status.hpp>

// DES-003 autonomy_node behavior (CAP-001 WP-1, task T1.2).
// Implements: MAP-6
class AutonomyNode : public rclcpp::Node
{
public:
    AutonomyNode();

    // DES-003 autonomy_node behavior, item 4 — lifecycle hooks. No WP-1
    // subscription drives these yet (mission progress monitoring and
    // failsafe/mode-change plumbing are out of CAP-001 scope); they are
    // public so a later work package can call them without touching the
    // dispatch path, and so tests can exercise the publish behavior directly.
    void onMissionComplete(const std::string & mission_id);
    void onMissionAborted(const std::string & mission_id, const std::string & reason);

private:
    void handleSurveyRequest(const drone_autonomy_msgs::msg::Mission::SharedPtr request);
    void publishStatus(const std::string & mission_id, const std::string & state,
                        const std::string & detail);
    static std::string makeMissionId();

    rclcpp::Publisher<drone_autonomy_msgs::msg::Mission>::SharedPtr mission_publisher_;
    rclcpp::Publisher<drone_autonomy_msgs::msg::MissionStatus>::SharedPtr mission_status_publisher_;
    rclcpp::Subscription<drone_autonomy_msgs::msg::Mission>::SharedPtr survey_request_subscriber_;
};
