// TP-002 TS-07 — Dataset format & offload verifiability, writer side
// (Verifies: MAP-3).
//
// Exercises mapping::DatasetWriter (pure logic, no ROS graph) directly:
//   (a) build a dataset the way survey_recorder_node would and check the
//       DES-004 schema: format_version, mission_id, survey fields, camera
//       intrinsics, frame_count/dropped_sync, frame_idx contiguity, and
//       that every recorded file's sha256 in manifest.yaml matches an
//       independently recomputed sha256 of that file;
//   (b) corrupt one image byte after finalize() and confirm the recomputed
//       sha256 no longer matches the manifest's recorded checksum for that
//       file (this is the writer-side half of TS-07's checksum-mismatch
//       check; tools/photogrammetry/verify_dataset.py, WP-3, is the CLI
//       that performs this check against a real exit code / naming the
//       corrupt file);
//   (c) confirm the writer never creates manifest.yaml except via the
//       atomic rename inside finalize() — i.e. a dataset that crashes
//       before finalize() has no manifest.yaml, which is DES-004's
//       definition of an invalid dataset ("a dataset without a final
//       manifest is invalid").
//
// Also exercises the sha256 implementation against FIPS 180-4 known-answer
// vectors, and the D7 free-space guard.
//
// Verifies: MAP-3

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "mapping/dataset_writer.hpp"

namespace fs = std::filesystem;

namespace
{

using mapping::CameraIntrinsics;
using mapping::DatasetWriter;
using mapping::FrameRecord;
using mapping::MissionSnapshot;

// Test fixture: a fresh temp directory per test, cleaned up afterward.
class DatasetWriterTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        tmp_root_ = fs::temp_directory_path() /
            fs::path("dataset_writer_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                      "_" + test_info_name());
        fs::create_directories(tmp_root_);
    }

    void TearDown() override
    {
        std::error_code ec;
        fs::remove_all(tmp_root_, ec);
    }

    std::string test_info_name() const
    {
        const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
        return std::string(info->test_suite_name()) + "_" + info->name();
    }

    // A JPEG-shaped (but not real-JPEG) byte buffer is fine — the writer
    // treats it as an opaque blob.
    static std::vector<uint8_t> fakeJpeg(int seed)
    {
        std::vector<uint8_t> bytes{0xFF, 0xD8, 0xFF, 0xE0};
        for (int i = 0; i < 100; ++i) {
            bytes.push_back(static_cast<uint8_t>((seed * 7 + i) % 256));
        }
        return bytes;
    }

    static FrameRecord makeFrame(int64_t stamp_ns, double sync_err_ms, bool has_gnss = true)
    {
        FrameRecord rec;
        rec.jpeg_bytes = fakeJpeg(static_cast<int>(stamp_ns));
        rec.stamp_ns = stamp_ns;
        rec.x = 1.5;
        rec.y = 2.5;
        rec.z = 40.0;
        rec.qx = 0.0;
        rec.qy = 0.0;
        rec.qz = 0.0;
        rec.qw = 1.0;
        rec.pose_stamp_ns = stamp_ns - static_cast<int64_t>(sync_err_ms * 1e6);
        rec.sync_err_ms = sync_err_ms;
        rec.has_gnss = has_gnss;
        if (has_gnss) {
            rec.lat = 37.421;
            rec.lon = -122.084;
            rec.alt_amsl = 45.0;
            rec.gnss_stamp_ns = stamp_ns;
        }
        return rec;
    }

    static MissionSnapshot makeSnapshot()
    {
        MissionSnapshot snap;
        snap.mission_id = "survey_2026-07-05T12:00:00.000Z";
        snap.polygon = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 80.0}, {0.0, 80.0}};
        snap.altitude_m = 40.0;
        snap.forward_overlap = 0.75;
        snap.side_overlap = 0.60;
        snap.capture_rate_hz = 2.0;
        return snap;
    }

    static CameraIntrinsics makeIntrinsics()
    {
        CameraIntrinsics intr;
        intr.valid = true;
        intr.width = 1280;
        intr.height = 720;
        intr.distortion_model = "plumb_bob";
        intr.d = {0.01, -0.02, 0.0, 0.0, 0.0};
        intr.k = {900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0};
        return intr;
    }

    // Minimal line-based reader for our own fixed manifest.yaml schema
    // (avoids pulling in a YAML parser just for this test; the real
    // consumer is PyYAML in tools/photogrammetry/verify_dataset.py, WP-3).
    static std::map<std::string, std::string> readManifestScalars(const std::string & path)
    {
        std::map<std::string, std::string> out;
        std::ifstream in(path);
        std::string line;
        while (std::getline(in, line)) {
            const auto pos = line.find(": ");
            if (pos == std::string::npos || line.rfind("  ", 0) == 0 || line.rfind("    ", 0) == 0) {
                continue;
            }
            out[line.substr(0, pos)] = line.substr(pos + 2);
        }
        return out;
    }

    // Parses the "files:" section into relative-path -> sha256.
    static std::map<std::string, std::string> readManifestFiles(const std::string & path)
    {
        std::map<std::string, std::string> out;
        std::ifstream in(path);
        std::string line;
        bool in_files = false;
        while (std::getline(in, line)) {
            if (line == "files:") {
                in_files = true;
                continue;
            }
            if (in_files) {
                if (line.empty() || line[0] != ' ') {
                    break;
                }
                const auto colon = line.find(": ");
                if (colon == std::string::npos) {
                    continue;
                }
                std::string key = line.substr(2, colon - 2);
                std::string val = line.substr(colon + 2);
                // Strip surrounding quotes.
                if (key.size() >= 2 && key.front() == '"' && key.back() == '"') {
                    key = key.substr(1, key.size() - 2);
                }
                if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
                    val = val.substr(1, val.size() - 2);
                }
                out[key] = val;
            }
        }
        return out;
    }

    static std::vector<std::vector<std::string>> readCsvRows(const std::string & path)
    {
        std::vector<std::vector<std::string>> rows;
        std::ifstream in(path);
        std::string line;
        bool first = true;
        while (std::getline(in, line)) {
            if (first) {
                first = false;
                continue;  // header
            }
            if (line.empty()) {
                continue;
            }
            std::vector<std::string> fields;
            std::stringstream ss(line);
            std::string field;
            while (std::getline(ss, field, ',')) {
                fields.push_back(field);
            }
            // Trailing empty field (e.g. row ends with ",") is dropped by
            // getline-on-',' — pad back out to 15 columns.
            while (fields.size() < 15) {
                fields.push_back("");
            }
            rows.push_back(fields);
        }
        return rows;
    }

    fs::path tmp_root_;
};

// ---------------------------------------------------------------------
// sha256 known-answer tests (FIPS 180-4 test vectors).
// ---------------------------------------------------------------------
TEST(Sha256KnownAnswer, EmptyString)
{
    EXPECT_EQ(mapping::sha256Bytes({}),
              "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256KnownAnswer, Abc)
{
    const std::vector<uint8_t> abc{'a', 'b', 'c'};
    EXPECT_EQ(mapping::sha256Bytes(abc),
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015a"
              "d");
}

TEST_F(DatasetWriterTest, Sha256FileMatchesBytes)
{
    const std::string path = (tmp_root_ / "blob.bin").string();
    std::ofstream(path, std::ios::binary) << "abc";
    EXPECT_EQ(mapping::sha256File(path), mapping::sha256Bytes({'a', 'b', 'c'}));
}

// ---------------------------------------------------------------------
// TS-07(a): schema + per-file checksums + frame_idx contiguity.
// ---------------------------------------------------------------------
TEST_F(DatasetWriterTest, WritesFullSchemaWithMatchingChecksumsAndContiguousFrameIdx)
{
    DatasetWriter writer(tmp_root_.string(), "survey_2026-07-05T12:00:00.000Z");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;

    writer.setMissionSnapshot(makeSnapshot());
    writer.setCameraIntrinsics(makeIntrinsics());

    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(writer.addFrame(makeFrame(1000000000LL + i * 500000000LL, 12.5 + i), err)) << err;
    }
    writer.recordDroppedSync();
    writer.recordDroppedSync();

    EXPECT_EQ(writer.frameCount(), 5);
    EXPECT_EQ(writer.droppedSyncCount(), 2);

    ASSERT_TRUE(writer.finalize("complete", err)) << err;

    const std::string manifest_path = (fs::path(writer.datasetDir()) / "manifest.yaml").string();
    const std::string part_path = (fs::path(writer.datasetDir()) / "manifest.yaml.part").string();
    ASSERT_TRUE(fs::exists(manifest_path)) << "manifest.yaml must exist after finalize()";
    EXPECT_FALSE(fs::exists(part_path)) << "manifest.yaml.part must be renamed away, not left behind";

    // Dataset directory name follows DES-004's survey_<mission_id>/ pattern.
    EXPECT_EQ(fs::path(writer.datasetDir()).filename().string(), "survey_survey_2026-07-05T12:00:00.000Z");

    const auto scalars = readManifestScalars(manifest_path);
    EXPECT_EQ(scalars.at("format_version"), "1");
    EXPECT_EQ(scalars.at("mission_id"), "\"survey_2026-07-05T12:00:00.000Z\"");
    EXPECT_EQ(scalars.at("status"), "\"complete\"");
    EXPECT_EQ(scalars.at("frame_count"), "5");
    EXPECT_EQ(scalars.at("dropped_sync"), "2");
    ASSERT_TRUE(scalars.count("start_utc"));
    ASSERT_TRUE(scalars.count("end_utc"));

    // Per-file sha256 present for poses.csv and every image, and each one
    // matches an independently recomputed checksum of the file on disk.
    const auto files = readManifestFiles(manifest_path);
    ASSERT_TRUE(files.count("poses.csv"));
    EXPECT_EQ(files.at("poses.csv"), mapping::sha256File((fs::path(writer.datasetDir()) / "poses.csv").string()));

    int image_count = 0;
    for (const auto & [rel, digest] : files) {
        if (rel == "poses.csv") {
            continue;
        }
        ++image_count;
        const std::string abs = (fs::path(writer.datasetDir()) / rel).string();
        ASSERT_TRUE(fs::exists(abs)) << rel << " listed in manifest but missing on disk";
        EXPECT_EQ(digest, mapping::sha256File(abs)) << "checksum mismatch for " << rel;
    }
    EXPECT_EQ(image_count, 5);

    // frame_idx contiguity (TS-07 "frame_idx contiguous").
    const auto rows = readCsvRows((fs::path(writer.datasetDir()) / "poses.csv").string());
    ASSERT_EQ(rows.size(), 5u);
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(rows[static_cast<size_t>(i)][0], std::to_string(i)) << "frame_idx not contiguous at row " << i;
    }

    // sync_err_ms recorded per row (D4 / TS-05 pass criterion re-verified here structurally).
    for (int i = 0; i < 5; ++i) {
        const double expected = 12.5 + i;
        const double got = std::stod(rows[static_cast<size_t>(i)][14]);
        EXPECT_NEAR(got, expected, 1e-6);
    }
}

TEST_F(DatasetWriterTest, GnssColumnsEmptyWhenNoFixWithinWindow)
{
    DatasetWriter writer(tmp_root_.string(), "no_gnss_mission");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;
    writer.setMissionSnapshot(makeSnapshot());

    ASSERT_TRUE(writer.addFrame(makeFrame(1000000000LL, 5.0, /*has_gnss=*/false), err)) << err;
    ASSERT_TRUE(writer.finalize("complete", err)) << err;

    const auto rows = readCsvRows((fs::path(writer.datasetDir()) / "poses.csv").string());
    ASSERT_EQ(rows.size(), 1u);
    // lat, lon, alt_amsl, gnss_stamp_ns are columns 10-13 (0-indexed).
    EXPECT_EQ(rows[0][10], "");
    EXPECT_EQ(rows[0][11], "");
    EXPECT_EQ(rows[0][12], "");
    EXPECT_EQ(rows[0][13], "");
}

TEST_F(DatasetWriterTest, ManifestOmitsCameraInfoSectionWhenIntrinsicsNotSet)
{
    DatasetWriter writer(tmp_root_.string(), "no_camera_info_mission");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;
    writer.setMissionSnapshot(makeSnapshot());
    ASSERT_TRUE(writer.addFrame(makeFrame(1000000000LL, 5.0), err)) << err;
    ASSERT_TRUE(writer.finalize("complete", err)) << err;

    std::ifstream in((fs::path(writer.datasetDir()) / "manifest.yaml").string());
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_EQ(contents.find("camera_info:"), std::string::npos);
}

// ---------------------------------------------------------------------
// TS-07(b): corrupted image is detectable via checksum mismatch.
// ---------------------------------------------------------------------
TEST_F(DatasetWriterTest, CorruptedImageChecksumMismatchIsDetectable)
{
    DatasetWriter writer(tmp_root_.string(), "corrupt_test_mission");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;
    writer.setMissionSnapshot(makeSnapshot());
    ASSERT_TRUE(writer.addFrame(makeFrame(1000000000LL, 3.0), err)) << err;
    ASSERT_TRUE(writer.finalize("complete", err)) << err;

    const auto files = readManifestFiles((fs::path(writer.datasetDir()) / "manifest.yaml").string());
    std::string image_rel;
    for (const auto & [rel, digest] : files) {
        if (rel != "poses.csv") {
            image_rel = rel;
            break;
        }
    }
    ASSERT_FALSE(image_rel.empty());
    const std::string image_abs = (fs::path(writer.datasetDir()) / image_rel).string();
    const std::string recorded_digest = files.at(image_rel);

    // Corrupt one byte.
    {
        std::fstream f(image_abs, std::ios::in | std::ios::out | std::ios::binary);
        ASSERT_TRUE(f);
        f.seekp(0);
        char c;
        f.get(c);
        f.seekp(0);
        f.put(static_cast<char>(c ^ 0xFF));
    }

    EXPECT_NE(mapping::sha256File(image_abs), recorded_digest)
        << "corrupting a byte must change the file's sha256, so verify_dataset.py (WP-3) can name "
        << image_rel << " as corrupt";
}

// ---------------------------------------------------------------------
// TS-07(c): a dataset that never finalizes has no manifest.yaml — invalid
// by DES-004's definition — even though poses.csv/images already exist.
// ---------------------------------------------------------------------
TEST_F(DatasetWriterTest, NoManifestUntilFinalizeIsCalled)
{
    DatasetWriter writer(tmp_root_.string(), "crash_before_finalize_mission");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;
    writer.setMissionSnapshot(makeSnapshot());
    ASSERT_TRUE(writer.addFrame(makeFrame(1000000000LL, 5.0), err)) << err;

    // Simulate a crash: writer goes out of scope without finalize().
    const std::string dataset_dir = writer.datasetDir();
    EXPECT_TRUE(fs::exists(fs::path(dataset_dir) / "poses.csv"));
    EXPECT_FALSE(fs::exists(fs::path(dataset_dir) / "manifest.yaml"))
        << "DES-004: a dataset without a finalized manifest is invalid";
    EXPECT_FALSE(fs::exists(fs::path(dataset_dir) / "manifest.yaml.part"));
}

TEST_F(DatasetWriterTest, TruncatedStatusRecordedOnErrorPath)
{
    DatasetWriter writer(tmp_root_.string(), "truncated_mission");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;
    writer.setMissionSnapshot(makeSnapshot());
    ASSERT_TRUE(writer.addFrame(makeFrame(1000000000LL, 5.0), err)) << err;

    // DES-004 "Failure behavior": disk-full/write error -> finalize with
    // status: truncated.
    ASSERT_TRUE(writer.finalize("truncated", err)) << err;

    const auto scalars = readManifestScalars((fs::path(writer.datasetDir()) / "manifest.yaml").string());
    EXPECT_EQ(scalars.at("status"), "\"truncated\"");
}

TEST_F(DatasetWriterTest, RejectsFrameOverSyncBudget)
{
    DatasetWriter writer(tmp_root_.string(), "over_budget_mission");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;

    // D4: pairs with sync_err_ms > 50 must not be added as frames.
    EXPECT_FALSE(writer.addFrame(makeFrame(1000000000LL, 50.001), err));
    EXPECT_EQ(writer.frameCount(), 0);
}

TEST_F(DatasetWriterTest, FinalizeIsNotIdempotent)
{
    DatasetWriter writer(tmp_root_.string(), "double_finalize_mission");
    std::string err;
    ASSERT_TRUE(writer.open(err)) << err;
    writer.setMissionSnapshot(makeSnapshot());
    ASSERT_TRUE(writer.finalize("complete", err)) << err;
    EXPECT_FALSE(writer.finalize("complete", err));
}

// ---------------------------------------------------------------------
// D7 free-space guard.
// ---------------------------------------------------------------------
TEST_F(DatasetWriterTest, FreeSpaceGuardPassesForModestRequirement)
{
    long free_mb = 0;
    EXPECT_TRUE(mapping::hasEnoughFreeSpace(tmp_root_.string(), 1, free_mb));
    EXPECT_GT(free_mb, 0);
}

TEST_F(DatasetWriterTest, FreeSpaceGuardFailsForAbsurdRequirement)
{
    long free_mb = 0;
    // No real filesystem has an exabyte free — this exercises D7's "refuse
    // to arm below min_free_mb" path without depending on how full the test
    // machine's disk happens to be.
    EXPECT_FALSE(mapping::hasEnoughFreeSpace(tmp_root_.string(), 1'000'000'000'000L, free_mb));
}

TEST_F(DatasetWriterTest, FreeSpaceGuardWorksBeforeDatasetDirExists)
{
    // D7 is checked before arming, i.e. before survey_<mission_id>/ is
    // created — the guard must walk up to an existing ancestor.
    const std::string not_yet_created = (tmp_root_ / "survey_future_mission").string();
    long free_mb = 0;
    EXPECT_TRUE(mapping::hasEnoughFreeSpace(not_yet_created, 1, free_mb));
    EXPECT_GT(free_mb, 0);
}

}  // namespace
