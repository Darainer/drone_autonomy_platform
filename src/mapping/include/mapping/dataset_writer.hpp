#pragma once

// Pure-logic survey dataset writer — DES-004 "Dataset format" section.
//
// No ROS dependencies: operates on plain types (bytes, doubles, strings) so
// it is unit-testable in isolation (TP-002 TS-07, writer side) without a ROS
// graph. survey_recorder_node wraps this class to convert
// sensor_msgs/geometry_msgs/drone_autonomy_msgs to and from these plain
// types, encode JPEG (D2), and evaluate the sync/rate/arm state machine.
//
// Dataset layout written by this class (DES-004):
//   survey_<mission_id>/
//     manifest.yaml     # written last, atomically (manifest.yaml.part -> manifest.yaml)
//     images/<frame_idx:06d>_<stamp_ns>.jpg
//     poses.csv         # frame_idx,stamp_ns,x,y,z,qx,qy,qz,qw,pose_stamp_ns,
//                        # lat,lon,alt_amsl,gnss_stamp_ns,sync_err_ms
//
// A dataset without a finalized manifest.yaml is invalid by definition
// (DES-004) — this class never creates manifest.yaml except via the atomic
// rename in finalize().
//
// Implements: MAP-3

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace mapping
{

// Camera intrinsics snapshot taken from /oak/rgb/camera_info at arm time
// (DES-004 survey_recorder_node behavior — "camera_info: latched capture at
// arm"). `valid` is false if no camera_info had arrived yet when the mission
// armed; manifest.yaml still gets written, with the camera_info section
// omitted in that case.
struct CameraIntrinsics
{
    bool valid{false};
    uint32_t width{0};
    uint32_t height{0};
    std::string distortion_model;
    std::vector<double> d;        // distortion coefficients, as-received
    std::array<double, 9> k{};    // 3x3 intrinsic matrix, row-major
};

// Mission fields copied verbatim into manifest.yaml (DES-004: "polygon,
// altitude, overlaps (copied from Mission)").
struct MissionSnapshot
{
    std::string mission_id;
    std::vector<std::pair<double, double>> polygon;  // survey_polygon vertices (x, y); z ignored
    double altitude_m{0.0};
    double forward_overlap{0.0};
    double side_overlap{0.0};
    // Effective capture rate actually used for this recording — i.e. the D5
    // fallback (default_capture_rate_hz) already resolved by the caller when
    // the Mission left capture_rate_hz at 0. Not the raw Mission field.
    double capture_rate_hz{0.0};
};

// One recorded frame + synchronized pose (+ optional GNSS) — becomes one
// poses.csv row and one images/ file. Populated by survey_recorder_node
// after ApproximateTime sync (D4) and rate limiting (D5) have both passed.
struct FrameRecord
{
    std::vector<uint8_t> jpeg_bytes;  // already-encoded JPEG (D2, quality 95 by default)

    int64_t stamp_ns{0};  // image header stamp (frame's canonical timestamp)
    double x{0.0}, y{0.0}, z{0.0};
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
    int64_t pose_stamp_ns{0};

    bool has_gnss{false};  // false if no GNSS fix within the 200 ms attach window (D4)
    double lat{0.0};
    double lon{0.0};
    double alt_amsl{0.0};
    int64_t gnss_stamp_ns{0};

    // |stamp_ns - pose_stamp_ns| in milliseconds (D4). Caller is expected to
    // have already dropped (and counted via recordDroppedSync()) any pair
    // exceeding the 50 ms budget rather than pass it to addFrame(); addFrame()
    // re-validates this defensively (see .cpp).
    double sync_err_ms{0.0};
};

// D7 — guard the external data volume backing `path` (the output_dir):
// refuse to arm below `min_free_mb` free space on it, and refuse to arm if
// that volume isn't mounted at all. Pure/testable: statvfs-based, no ROS
// deps. `path` may not exist yet (survey_<mission_id>/ is created lazily),
// so the check looks at `path` itself and, failing that, exactly one level
// up (the mount point) — never further, so an unmounted volume can't fall
// through to a real ancestor filesystem (e.g. `/`) and pass against the
// system disk instead. Returns false (guard tripped) if neither exists, or
// if the check itself fails (e.g. statvfs error) — fail closed. `free_mb_out`
// is always populated on success (0 otherwise).
bool hasEnoughFreeSpace(const std::string & path, long min_free_mb, long & free_mb_out);

// sha256 hex digest of a file's contents. Used for manifest.yaml checksums
// and by tests to independently verify them (TS-07).
std::string sha256File(const std::string & path);

// sha256 hex digest of an in-memory byte buffer (used internally, exposed
// for the sha256 known-answer tests).
std::string sha256Bytes(const std::vector<uint8_t> & data);

// Current UTC time as an ISO-8601 string with millisecond resolution
// ("YYYY-MM-DDTHH:MM:SS.mmmZ") — used for manifest.yaml start/end times.
std::string nowUtcIso8601();

// Writes one survey_<mission_id>/ dataset directory (DES-004 "Dataset
// format"). One instance per armed recording; survey_recorder_node
// constructs a fresh instance (and therefore a fresh directory) on every
// arm event — TS-06: "a second arm creates a new dataset directory, never
// appends."
class DatasetWriter
{
public:
    DatasetWriter(std::string output_dir, std::string mission_id);

    // Creates survey_<mission_id>/ and survey_<mission_id>/images/, and opens
    // poses.csv for writing (header row written immediately). Returns false
    // (with `error` populated) if the directories/file could not be created.
    bool open(std::string & error);

    // Absolute path to survey_<mission_id>/.
    const std::string & datasetDir() const { return dataset_dir_; }

    void setCameraIntrinsics(const CameraIntrinsics & intrinsics) { intrinsics_ = intrinsics; }
    void setMissionSnapshot(const MissionSnapshot & snapshot) { snapshot_ = snapshot; }

    // Writes images/<frame_idx:06d>_<stamp_ns>.jpg and appends one poses.csv
    // row. frame_idx is assigned by this method as 0, 1, 2, ... in call
    // order — contiguous by construction (TS-07: "frame_idx contiguous").
    // Returns false (with `error` populated, no frame_idx consumed) on any
    // write failure (e.g. disk full) — DES-004 "Failure behavior": the
    // caller must stop recording and finalize with status "truncated".
    bool addFrame(const FrameRecord & record, std::string & error);

    // D4 — records one sync'd pair rejected for exceeding the 50 ms slop
    // budget; counted in manifest.yaml "dropped_sync" without writing a
    // frame or consuming a frame_idx.
    void recordDroppedSync() { ++dropped_sync_; }

    // Finalizes the dataset: writes manifest.yaml.part (format_version,
    // mission_id, survey polygon/altitude/overlaps, start/end UTC,
    // frame_count, dropped_sync, camera intrinsics if available, sha256 per
    // file, status), then atomically renames it to manifest.yaml. `status`
    // is normally "complete" (normal disarm) or "truncated" (DES-004
    // Failure behavior: disk-full/write error). Returns false if the
    // manifest could not be written/renamed. Safe to call at most once;
    // subsequent calls return false without modifying an already-finalized
    // dataset.
    bool finalize(const std::string & status, std::string & error);

    int frameCount() const { return next_frame_idx_; }
    int droppedSyncCount() const { return dropped_sync_; }

private:
    std::string output_dir_;
    std::string mission_id_;
    std::string dataset_dir_;
    std::string start_utc_;

    int next_frame_idx_{0};
    int dropped_sync_{0};
    bool finalized_{false};

    CameraIntrinsics intrinsics_;
    MissionSnapshot snapshot_;

    // Relative-to-dataset-dir paths of every file written so far (poses.csv
    // + each image), in write order — hashed at finalize() time for the
    // manifest's per-file checksums.
    std::vector<std::string> written_files_;

    std::string posesCsvPath() const;
};

}  // namespace mapping
