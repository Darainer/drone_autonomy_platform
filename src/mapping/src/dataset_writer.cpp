#include "mapping/dataset_writer.hpp"

#include <sys/statvfs.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace mapping
{

namespace fs = std::filesystem;

// ---------------------------------------------------------------------
// sha256 — minimal self-contained implementation (FIPS 180-4), no external
// dependency so this package's pure-logic library stays free of anything
// beyond the C++ standard library and POSIX (statvfs).
// ---------------------------------------------------------------------
namespace
{

constexpr std::array<uint32_t, 64> kSha256K = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }

class Sha256Ctx
{
public:
    Sha256Ctx()
    {
        h_ = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
              0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    }

    void update(const uint8_t * data, size_t len)
    {
        total_len_ += len;
        while (len > 0) {
            const size_t take = std::min(len, static_cast<size_t>(64) - buf_len_);
            std::memcpy(buf_.data() + buf_len_, data, take);
            buf_len_ += take;
            data += take;
            len -= take;
            if (buf_len_ == 64) {
                processBlock(buf_.data());
                buf_len_ = 0;
            }
        }
    }

    std::string hexDigest()
    {
        // Padding: 0x80, zeros, then 64-bit big-endian bit length.
        const uint64_t bit_len = total_len_ * 8;
        uint8_t pad = 0x80;
        update(&pad, 1);
        uint8_t zero = 0x00;
        while (buf_len_ != 56) {
            update(&zero, 1);
        }
        std::array<uint8_t, 8> len_bytes{};
        for (int i = 0; i < 8; ++i) {
            len_bytes[7 - i] = static_cast<uint8_t>((bit_len >> (8 * i)) & 0xff);
        }
        // Bypass update() bookkeeping for the length field itself.
        std::memcpy(buf_.data() + 56, len_bytes.data(), 8);
        processBlock(buf_.data());

        std::ostringstream oss;
        oss << std::hex << std::setfill('0');
        for (uint32_t word : h_) {
            oss << std::setw(8) << word;
        }
        return oss.str();
    }

private:
    void processBlock(const uint8_t * block)
    {
        std::array<uint32_t, 64> w{};
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(block[i * 4]) << 24) |
                   (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
                   (static_cast<uint32_t>(block[i * 4 + 3]));
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3];
        uint32_t e = h_[4], f = h_[5], g = h_[6], hh = h_[7];

        for (int i = 0; i < 64; ++i) {
            const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t temp1 = hh + s1 + ch + kSha256K[i] + w[i];
            const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = s0 + maj;

            hh = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }

        h_[0] += a; h_[1] += b; h_[2] += c; h_[3] += d;
        h_[4] += e; h_[5] += f; h_[6] += g; h_[7] += hh;
    }

    std::array<uint32_t, 8> h_{};
    std::array<uint8_t, 64> buf_{};
    size_t buf_len_{0};
    uint64_t total_len_{0};
};

}  // namespace

std::string sha256Bytes(const std::vector<uint8_t> & data)
{
    Sha256Ctx ctx;
    if (!data.empty()) {
        ctx.update(data.data(), data.size());
    }
    return ctx.hexDigest();
}

std::string sha256File(const std::string & path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return "";
    }
    Sha256Ctx ctx;
    std::array<char, 64 * 1024> buf{};
    while (in.read(buf.data(), buf.size()) || in.gcount() > 0) {
        ctx.update(reinterpret_cast<const uint8_t *>(buf.data()), static_cast<size_t>(in.gcount()));
    }
    return ctx.hexDigest();
}

// ---------------------------------------------------------------------
// D7 free-space guard.
// ---------------------------------------------------------------------
bool hasEnoughFreeSpace(const std::string & path, long min_free_mb, long & free_mb_out)
{
    free_mb_out = 0;

    // statvfs requires an existing path; walk up to the nearest existing
    // ancestor so the guard still works before survey_<mission_id>/ exists.
    std::error_code ec;
    fs::path probe(path);
    while (!probe.empty() && !fs::exists(probe, ec)) {
        probe = probe.parent_path();
    }
    if (probe.empty()) {
        return false;  // fail closed: nothing on this path resolves to a real filesystem
    }

    struct statvfs st{};
    if (statvfs(probe.c_str(), &st) != 0) {
        return false;  // fail closed
    }

    const unsigned long long free_bytes =
        static_cast<unsigned long long>(st.f_bavail) * static_cast<unsigned long long>(st.f_frsize);
    free_mb_out = static_cast<long>(free_bytes / (1024ULL * 1024ULL));
    return free_mb_out >= min_free_mb;
}

// ---------------------------------------------------------------------
// Timestamps.
// ---------------------------------------------------------------------
std::string nowUtcIso8601()
{
    using namespace std::chrono;
    const auto now = system_clock::now();
    const std::time_t t = system_clock::to_time_t(now);
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000;

    std::tm utc_tm{};
    gmtime_r(&t, &utc_tm);

    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms << "Z";
    return oss.str();
}

namespace
{

// Minimal YAML scalar quoting: manifest.yaml has a fixed, known schema (this
// is the only writer), so a full YAML library is unnecessary — this quotes
// the handful of string scalars we ever emit (mission_id, distortion_model,
// status, ISO timestamps, relative file paths) so they round-trip through
// any standard YAML parser (e.g. PyYAML in tools/photogrammetry, WP-3).
std::string yamlQuoted(const std::string & s)
{
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += c;
    }
    out += "\"";
    return out;
}

// Full round-trip precision for doubles copied from the Mission / CameraInfo.
std::string yamlDouble(double v)
{
    std::ostringstream oss;
    oss << std::setprecision(17) << v;
    return oss.str();
}

std::string csvField(bool present, double v)
{
    if (!present) {
        return "";
    }
    std::ostringstream oss;
    oss << std::setprecision(17) << v;
    return oss.str();
}

}  // namespace

// ---------------------------------------------------------------------
// DatasetWriter.
// ---------------------------------------------------------------------
DatasetWriter::DatasetWriter(std::string output_dir, std::string mission_id)
: output_dir_(std::move(output_dir)), mission_id_(std::move(mission_id))
{
    // DES-004 "Dataset format": survey_<mission_id>/ under the configured
    // output_dir (external data volume, D7).
    dataset_dir_ = (fs::path(output_dir_) / ("survey_" + mission_id_)).string();
}

std::string DatasetWriter::posesCsvPath() const
{
    return (fs::path(dataset_dir_) / "poses.csv").string();
}

bool DatasetWriter::open(std::string & error)
{
    std::error_code ec;
    fs::create_directories(fs::path(dataset_dir_) / "images", ec);
    if (ec) {
        error = "failed to create dataset directory " + dataset_dir_ + ": " + ec.message();
        return false;
    }

    std::ofstream csv(posesCsvPath(), std::ios::out | std::ios::trunc);
    if (!csv) {
        error = "failed to create " + posesCsvPath();
        return false;
    }
    // DES-004 "Dataset format": poses.csv column order, exactly.
    csv << "frame_idx,stamp_ns,x,y,z,qx,qy,qz,qw,pose_stamp_ns,lat,lon,alt_amsl,gnss_stamp_ns,sync_err_ms\n";
    if (!csv) {
        error = "failed to write poses.csv header";
        return false;
    }
    csv.close();

    start_utc_ = nowUtcIso8601();
    written_files_.push_back("poses.csv");
    return true;
}

bool DatasetWriter::addFrame(const FrameRecord & record, std::string & error)
{
    if (finalized_) {
        error = "addFrame() called after finalize()";
        return false;
    }
    // Defensive re-check of the D4 sync budget: addFrame() is the single
    // place frames enter the dataset, so this holds even if a future caller
    // forgets the pre-check the node currently does.
    if (record.sync_err_ms > 50.0) {
        error = "sync_err_ms exceeds the 50 ms budget (D4); caller should use recordDroppedSync() instead";
        return false;
    }

    const int frame_idx = next_frame_idx_;

    std::ostringstream name;
    name << std::setfill('0') << std::setw(6) << frame_idx << "_" << record.stamp_ns << ".jpg";
    const std::string rel_path = (fs::path("images") / name.str()).string();
    const std::string abs_path = (fs::path(dataset_dir_) / rel_path).string();

    std::ofstream img(abs_path, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!img) {
        error = "failed to open " + abs_path + " for writing";
        return false;
    }
    if (!record.jpeg_bytes.empty()) {
        img.write(reinterpret_cast<const char *>(record.jpeg_bytes.data()),
                   static_cast<std::streamsize>(record.jpeg_bytes.size()));
    }
    img.flush();
    if (!img) {
        error = "write error while writing " + abs_path + " (disk full?)";
        return false;
    }
    img.close();

    std::ofstream csv(posesCsvPath(), std::ios::out | std::ios::app);
    if (!csv) {
        error = "failed to reopen poses.csv for append";
        return false;
    }
    csv << frame_idx << ',' << record.stamp_ns << ','
        << yamlDouble(record.x) << ',' << yamlDouble(record.y) << ',' << yamlDouble(record.z) << ','
        << yamlDouble(record.qx) << ',' << yamlDouble(record.qy) << ',' << yamlDouble(record.qz) << ','
        << yamlDouble(record.qw) << ',' << record.pose_stamp_ns << ','
        << csvField(record.has_gnss, record.lat) << ',' << csvField(record.has_gnss, record.lon) << ','
        << csvField(record.has_gnss, record.alt_amsl) << ','
        << (record.has_gnss ? std::to_string(record.gnss_stamp_ns) : std::string()) << ','
        << yamlDouble(record.sync_err_ms) << '\n';
    if (!csv) {
        error = "write error while appending poses.csv row (disk full?)";
        return false;
    }
    csv.flush();
    csv.close();

    written_files_.push_back(rel_path);
    next_frame_idx_ = frame_idx + 1;
    return true;
}

bool DatasetWriter::finalize(const std::string & status, std::string & error)
{
    if (finalized_) {
        error = "finalize() already called for this dataset";
        return false;
    }

    const std::string part_path = (fs::path(dataset_dir_) / "manifest.yaml.part").string();
    const std::string final_path = (fs::path(dataset_dir_) / "manifest.yaml").string();

    std::ofstream out(part_path, std::ios::out | std::ios::trunc);
    if (!out) {
        error = "failed to open " + part_path + " for writing";
        return false;
    }

    out << "format_version: 1\n";
    out << "mission_id: " << yamlQuoted(mission_id_) << "\n";
    out << "status: " << yamlQuoted(status) << "\n";
    out << "start_utc: " << yamlQuoted(start_utc_) << "\n";
    out << "end_utc: " << yamlQuoted(nowUtcIso8601()) << "\n";
    out << "frame_count: " << next_frame_idx_ << "\n";
    out << "dropped_sync: " << dropped_sync_ << "\n";
    out << "capture_rate_hz: " << yamlDouble(snapshot_.capture_rate_hz) << "\n";

    out << "survey:\n";
    out << "  altitude_m: " << yamlDouble(snapshot_.altitude_m) << "\n";
    out << "  forward_overlap: " << yamlDouble(snapshot_.forward_overlap) << "\n";
    out << "  side_overlap: " << yamlDouble(snapshot_.side_overlap) << "\n";
    out << "  polygon:\n";
    for (const auto & [px, py] : snapshot_.polygon) {
        out << "    - {x: " << yamlDouble(px) << ", y: " << yamlDouble(py) << "}\n";
    }

    if (intrinsics_.valid) {
        out << "camera_info:\n";
        out << "  width: " << intrinsics_.width << "\n";
        out << "  height: " << intrinsics_.height << "\n";
        out << "  distortion_model: " << yamlQuoted(intrinsics_.distortion_model) << "\n";
        out << "  d: [";
        for (size_t i = 0; i < intrinsics_.d.size(); ++i) {
            out << yamlDouble(intrinsics_.d[i]) << (i + 1 < intrinsics_.d.size() ? ", " : "");
        }
        out << "]\n";
        out << "  k: [";
        for (size_t i = 0; i < intrinsics_.k.size(); ++i) {
            out << yamlDouble(intrinsics_.k[i]) << (i + 1 < intrinsics_.k.size() ? ", " : "");
        }
        out << "]\n";
    }

    // Per-file sha256 (DES-004: "sha256 per file") — covers poses.csv and
    // every images/*.jpg written before this call.
    out << "files:\n";
    if (!out) {
        error = "write error while writing manifest.yaml.part (disk full?)";
        return false;
    }
    for (const std::string & rel : written_files_) {
        const std::string abs = (fs::path(dataset_dir_) / rel).string();
        const std::string digest = sha256File(abs);
        out << "  " << yamlQuoted(rel) << ": " << yamlQuoted(digest) << "\n";
        if (!out) {
            error = "write error while writing manifest.yaml.part checksums (disk full?)";
            return false;
        }
    }
    out.flush();
    if (!out) {
        error = "write error finalizing manifest.yaml.part (disk full?)";
        return false;
    }
    out.close();

    // Atomic rename (DES-004: "manifest.yaml is written on disarm (atomic
    // rename from manifest.yaml.part); a dataset without a final manifest is
    // invalid by definition").
    std::error_code ec;
    fs::rename(part_path, final_path, ec);
    if (ec) {
        error = "atomic rename of manifest.yaml.part to manifest.yaml failed: " + ec.message();
        return false;
    }

    finalized_ = true;
    return true;
}

}  // namespace mapping
