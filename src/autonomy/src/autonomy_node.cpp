#include "autonomy/autonomy_node.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "autonomy/survey_validation.hpp"

AutonomyNode::AutonomyNode() : Node("autonomy_node")
{
    // DES-003 Interfaces table: ~/mission stays reliable, depth 10 (existing,
    // unchanged by this design).
    mission_publisher_ = this->create_publisher<drone_autonomy_msgs::msg::Mission>("~/mission", 10);

    // DES-003 Interfaces table: ~/mission_status is reliable + transient_local
    // so a late-joining recorder (WP-2) still observes the current state.
    rclcpp::QoS mission_status_qos(rclcpp::KeepLast(10));
    mission_status_qos.reliable();
    mission_status_qos.transient_local();
    mission_status_publisher_ = this->create_publisher<drone_autonomy_msgs::msg::MissionStatus>(
        "~/mission_status", mission_status_qos);

    // DES-003 autonomy_node behavior, item 1: ~/survey_request, reliable,
    // depth 10 — the GCS-facing entry point for survey missions.
    survey_request_subscriber_ = this->create_subscription<drone_autonomy_msgs::msg::Mission>(
        "~/survey_request", rclcpp::QoS(10).reliable(),
        std::bind(&AutonomyNode::handleSurveyRequest, this, std::placeholders::_1));
}

void AutonomyNode::handleSurveyRequest(const drone_autonomy_msgs::msg::Mission::SharedPtr request)
{
    // Implements: MAP-6 — DES-003 autonomy_node behavior, items 2-3: validate
    // the incoming survey spec and either reject it or dispatch it.
    const auto result = survey_validation::validate_survey_mission(*request);
    if (!result.ok) {
        RCLCPP_WARN(this->get_logger(), "survey_request rejected: %s", result.detail.c_str());
        publishStatus(request->mission_id, "rejected", result.detail);
        return;
    }

    auto mission = *request;
    mission.mission_type = "survey";
    mission.mission_id = makeMissionId();
    mission_publisher_->publish(mission);

    RCLCPP_INFO(this->get_logger(), "dispatched survey mission %s", mission.mission_id.c_str());
    publishStatus(mission.mission_id, "active", "");
}

void AutonomyNode::onMissionComplete(const std::string & mission_id)
{
    // DES-003 autonomy_node behavior, item 4 — v1 timer-armed placeholder
    // hook; full completion tracking arrives with mission progress
    // monitoring (out of CAP-001 scope).
    publishStatus(mission_id, "complete", "");
}

void AutonomyNode::onMissionAborted(const std::string & mission_id, const std::string & reason)
{
    // DES-003 autonomy_node behavior, item 4 — placeholder hook for any
    // failsafe/mode-change input; not wired to a subscription in WP-1.
    publishStatus(mission_id, "aborted", reason);
}

void AutonomyNode::publishStatus(const std::string & mission_id, const std::string & state,
                                  const std::string & detail)
{
    auto status = drone_autonomy_msgs::msg::MissionStatus();
    status.header.stamp = this->now();
    status.mission_id = mission_id;
    status.state = state;
    status.detail = detail;
    mission_status_publisher_->publish(status);
}

std::string AutonomyNode::makeMissionId()
{
    // DES-003: "survey_<utc-iso>", millisecond resolution for uniqueness.
    using namespace std::chrono;
    const auto now = system_clock::now();
    const std::time_t t = system_clock::to_time_t(now);
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000;

    std::tm utc_tm{};
    gmtime_r(&t, &utc_tm);

    std::ostringstream oss;
    oss << "survey_" << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms << "Z";
    return oss.str();
}
