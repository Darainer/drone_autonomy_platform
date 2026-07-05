// survey_recorder_node — DES-004 "survey_recorder_node" section.
//
// Records camera frames with time-synchronized local pose and GNSS fix to a
// DES-004-format dataset (mapping::DatasetWriter), armed/disarmed by the
// survey mission lifecycle (D6). All ROS graph wiring (subscriptions, QoS,
// message_filters sync, JPEG encode) lives here; the dataset format itself
// (manifest.yaml, poses.csv, sha256 checksums) is pure logic in
// mapping::DatasetWriter (dataset_writer.hpp/.cpp) so it is unit-testable
// without a ROS graph (TP-002 TS-07, writer side).
//
// Implements: MAP-2

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <drone_autonomy_msgs/msg/mission.hpp>
#include <drone_autonomy_msgs/msg/mission_status.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include "mapping/dataset_writer.hpp"

namespace
{

// D4: GNSS attached as most-recent-within-200 ms of the image stamp.
constexpr int64_t kGnssAttachWindowNs = 200'000'000LL;
// D4: ApproximateTime(image, pose) sync budget; pairs exceeding this are
// dropped and counted (dropped_sync in manifest.yaml).
constexpr double kMaxSyncErrMs = 50.0;

mapping::CameraIntrinsics toIntrinsics(const sensor_msgs::msg::CameraInfo & info)
{
    mapping::CameraIntrinsics intr;
    intr.valid = true;
    intr.width = info.width;
    intr.height = info.height;
    intr.distortion_model = info.distortion_model;
    intr.d.assign(info.d.begin(), info.d.end());
    std::copy(info.k.begin(), info.k.end(), intr.k.begin());
    return intr;
}

}  // namespace

class SurveyRecorderNode : public rclcpp::Node
{
public:
    SurveyRecorderNode() : Node("survey_recorder_node")
    {
        // DES-004 survey_recorder_node behavior: declared parameters,
        // defaults exactly per the design doc.
        this->declare_parameter<double>("default_capture_rate_hz", 2.0);
        this->declare_parameter<std::string>("output_dir", "/data/surveys");
        this->declare_parameter<int>("jpeg_quality", 95);
        this->declare_parameter<int>("min_free_mb", 500);

        default_capture_rate_hz_ = this->get_parameter("default_capture_rate_hz").as_double();
        output_dir_ = this->get_parameter("output_dir").as_string();
        jpeg_quality_ = static_cast<int>(this->get_parameter("jpeg_quality").as_int());
        min_free_mb_ = this->get_parameter("min_free_mb").as_int();

        // --- Interfaces table: subscriptions with the exact QoS per row. ---

        // /oak/rgb/camera_info — reliable, depth 1: intrinsics snapshot at arm.
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/oak/rgb/camera_info", rclcpp::QoS(1).reliable(),
            std::bind(&SurveyRecorderNode::cameraInfoCallback, this, std::placeholders::_1));

        // /oak/rgb/camera_info info is latched: keep only the most recent.

        // /mavros/global_position/global — best_effort, depth 10.
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", rclcpp::QoS(10).best_effort(),
            std::bind(&SurveyRecorderNode::gnssCallback, this, std::placeholders::_1));

        // /mission — reliable, depth 10: arm trigger.
        mission_sub_ = this->create_subscription<drone_autonomy_msgs::msg::Mission>(
            "/mission", rclcpp::QoS(10).reliable(),
            std::bind(&SurveyRecorderNode::missionCallback, this, std::placeholders::_1));

        // /mission_status — reliable, transient_local, depth 10: disarm
        // trigger; transient_local so a late-joining node still observes the
        // current state (matches autonomy_node's publisher QoS, DES-003).
        rclcpp::QoS mission_status_qos(rclcpp::KeepLast(10));
        mission_status_qos.reliable();
        mission_status_qos.transient_local();
        mission_status_sub_ = this->create_subscription<drone_autonomy_msgs::msg::MissionStatus>(
            "/mission_status", mission_status_qos,
            std::bind(&SurveyRecorderNode::missionStatusCallback, this, std::placeholders::_1));

        // /oak/rgb/image_raw — sensor-data profile (best_effort, depth 5) —
        // and /mavros/local_position/pose — best_effort, depth 30 — feed the
        // D4 ApproximateTime(image, pose) sync, queue 30.
        image_sub_.subscribe(this, "/oak/rgb/image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        pose_sub_.subscribe(
            this, "/mavros/local_position/pose", rclcpp::QoS(30).best_effort().get_rmw_qos_profile());
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(30), image_sub_, pose_sub_);
        sync_->registerCallback(
            std::bind(&SurveyRecorderNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "survey_recorder_node ready (output_dir=%s, min_free_mb=%ld)",
                    output_dir_.c_str(), min_free_mb_);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>;

    // --- D6: arm/disarm trigger. ---

    void missionCallback(const drone_autonomy_msgs::msg::Mission::SharedPtr msg)
    {
        if (msg->mission_type != "survey") {
            return;
        }
        if (armed_) {
            RCLCPP_WARN(this->get_logger(),
                        "already recording mission %s; ignoring arm for %s (overlapping survey missions "
                        "are not supported in v1)",
                        current_mission_id_.c_str(), msg->mission_id.c_str());
            return;
        }

        // D7: refuse to arm below min_free_mb free space on output_dir's volume.
        long free_mb = 0;
        if (!mapping::hasEnoughFreeSpace(output_dir_, min_free_mb_, free_mb)) {
            RCLCPP_ERROR(this->get_logger(),
                         "refusing to arm for mission %s: %ld MB free on %s (< min_free_mb=%ld)",
                         msg->mission_id.c_str(), free_mb, output_dir_.c_str(), min_free_mb_);
            return;
        }

        // D5: capture rate from the arming Mission, falling back to
        // default_capture_rate_hz only when the mission leaves it at 0.
        capture_rate_hz_ = (msg->capture_rate_hz > 0.0f)
            ? static_cast<double>(msg->capture_rate_hz)
            : default_capture_rate_hz_;

        writer_ = std::make_unique<mapping::DatasetWriter>(output_dir_, msg->mission_id);
        std::string err;
        if (!writer_->open(err)) {
            RCLCPP_ERROR(this->get_logger(), "failed to open dataset for mission %s: %s",
                         msg->mission_id.c_str(), err.c_str());
            writer_.reset();
            return;
        }

        mapping::MissionSnapshot snapshot;
        snapshot.mission_id = msg->mission_id;
        snapshot.polygon.reserve(msg->survey_polygon.points.size());
        for (const auto & pt : msg->survey_polygon.points) {
            snapshot.polygon.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y));
        }
        snapshot.altitude_m = static_cast<double>(msg->survey_altitude_m);
        snapshot.forward_overlap = static_cast<double>(msg->forward_overlap);
        snapshot.side_overlap = static_cast<double>(msg->side_overlap);
        snapshot.capture_rate_hz = capture_rate_hz_;
        writer_->setMissionSnapshot(snapshot);

        if (have_camera_info_) {
            writer_->setCameraIntrinsics(toIntrinsics(latest_camera_info_));
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "arming mission %s with no camera_info received yet; manifest will omit intrinsics",
                        msg->mission_id.c_str());
        }

        current_mission_id_ = msg->mission_id;
        armed_ = true;
        have_last_capture_time_ = false;

        RCLCPP_INFO(this->get_logger(), "armed recording for survey mission %s at %.2f Hz -> %s",
                    current_mission_id_.c_str(), capture_rate_hz_, writer_->datasetDir().c_str());
    }

    void missionStatusCallback(const drone_autonomy_msgs::msg::MissionStatus::SharedPtr msg)
    {
        if (!armed_ || msg->mission_id != current_mission_id_) {
            return;
        }
        if (msg->state != "complete" && msg->state != "aborted") {
            return;
        }

        finalizeRecording(msg->state);
    }

    // --- D4/D5: sync, budget check, rate limiting, encode, write. ---

    void syncedCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image,
                         const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose)
    {
        if (!armed_) {
            return;
        }

        const int64_t stamp_img_ns = rclcpp::Time(image->header.stamp).nanoseconds();
        const int64_t stamp_pose_ns = rclcpp::Time(pose->header.stamp).nanoseconds();
        const double sync_err_ms =
            std::fabs(static_cast<double>(stamp_img_ns - stamp_pose_ns)) / 1.0e6;

        if (sync_err_ms > kMaxSyncErrMs) {
            writer_->recordDroppedSync();
            return;
        }

        // D5: rate limiting after sync (drop, don't queue) to the effective
        // capture rate resolved at arm time.
        const rclcpp::Time image_time(image->header.stamp);
        if (have_last_capture_time_) {
            const double min_period_s = 1.0 / capture_rate_hz_;
            if ((image_time - last_capture_time_).seconds() < min_period_s) {
                return;
            }
        }
        last_capture_time_ = image_time;
        have_last_capture_time_ = true;

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed, dropping frame: %s", e.what());
            return;
        }

        // D2: JPEG quality 95 (parameterized as jpeg_quality).
        std::vector<uint8_t> jpeg_bytes;
        const std::vector<int> encode_params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        if (!cv::imencode(".jpg", cv_ptr->image, jpeg_bytes, encode_params)) {
            RCLCPP_ERROR(this->get_logger(), "JPEG encode failed, dropping frame");
            return;
        }

        mapping::FrameRecord record;
        record.jpeg_bytes = std::move(jpeg_bytes);
        record.stamp_ns = stamp_img_ns;
        record.x = pose->pose.position.x;
        record.y = pose->pose.position.y;
        record.z = pose->pose.position.z;
        record.qx = pose->pose.orientation.x;
        record.qy = pose->pose.orientation.y;
        record.qz = pose->pose.orientation.z;
        record.qw = pose->pose.orientation.w;
        record.pose_stamp_ns = stamp_pose_ns;
        record.sync_err_ms = sync_err_ms;

        // D4: GNSS attached as most-recent-within-200 ms of the image stamp.
        if (have_gnss_ && std::llabs(stamp_img_ns - latest_gnss_stamp_ns_) <= kGnssAttachWindowNs) {
            record.has_gnss = true;
            record.lat = latest_gnss_.latitude;
            record.lon = latest_gnss_.longitude;
            record.alt_amsl = latest_gnss_.altitude;
            record.gnss_stamp_ns = latest_gnss_stamp_ns_;
        }

        std::string err;
        if (!writer_->addFrame(record, err)) {
            // DES-004 "Failure behavior": disk-full/write error -> stop
            // recording, log ERROR, finalize with status: truncated.
            RCLCPP_ERROR(this->get_logger(), "dataset write failed for mission %s: %s — stopping recording",
                         current_mission_id_.c_str(), err.c_str());
            finalizeRecording("truncated");
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        latest_camera_info_ = *msg;
        have_camera_info_ = true;
    }

    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        latest_gnss_ = *msg;
        latest_gnss_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
        have_gnss_ = true;
    }

    void finalizeRecording(const std::string & status)
    {
        if (!writer_) {
            return;
        }
        std::string err;
        if (!writer_->finalize(status, err)) {
            RCLCPP_ERROR(this->get_logger(), "failed to finalize dataset for mission %s: %s",
                         current_mission_id_.c_str(), err.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "finalized dataset for mission %s (%s): %d frames, %d dropped_sync",
                        current_mission_id_.c_str(), status.c_str(), writer_->frameCount(),
                        writer_->droppedSyncCount());
        }
        writer_.reset();
        armed_ = false;
        current_mission_id_.clear();
    }

    // Parameters.
    double default_capture_rate_hz_{2.0};
    std::string output_dir_;
    int jpeg_quality_{95};
    long min_free_mb_{500};

    // Arm state (D6).
    bool armed_{false};
    std::string current_mission_id_;
    double capture_rate_hz_{2.0};
    std::unique_ptr<mapping::DatasetWriter> writer_;
    bool have_last_capture_time_{false};
    rclcpp::Time last_capture_time_;

    // Latched camera_info snapshot.
    bool have_camera_info_{false};
    sensor_msgs::msg::CameraInfo latest_camera_info_;

    // Latest GNSS fix (D4 attach window).
    bool have_gnss_{false};
    sensor_msgs::msg::NavSatFix latest_gnss_;
    int64_t latest_gnss_stamp_ns_{0};

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<drone_autonomy_msgs::msg::Mission>::SharedPtr mission_sub_;
    rclcpp::Subscription<drone_autonomy_msgs::msg::MissionStatus>::SharedPtr mission_status_sub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurveyRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
