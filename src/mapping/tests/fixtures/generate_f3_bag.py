#!/usr/bin/env python3
"""Generate the TP-002 "Reference fixtures" F3 replay bag.

CAP-001 WP-2 T2.3. Writes a rosbag2 (storage_id='sqlite3') recording of the
three sensor topics survey_recorder_node consumes for its ApproximateTime
sync + GNSS attach (DES-004 D4):

    /oak/rgb/image_raw          sensor_msgs/msg/Image           15 Hz
    /mavros/local_position/pose geometry_msgs/msg/PoseStamped    30 Hz
    /mavros/global_position/global sensor_msgs/msg/NavSatFix      5 Hz

F3 carries ONLY these sensor topics -- test_survey_recorder_replay.py itself
publishes /mission and /mission_status live during replay to drive the D6
arm/disarm sequence.

There is no ROS 2 installation in this sandbox (no rosbag2_py, no rclpy
serialization), so this script hand-rolls the two pieces ROS 2 would
otherwise provide:

  1. CDR (Common Data Representation) serialization of the three message
     types, byte-for-byte per the OMG CDR alignment rules ROS 2's default
     (non-XTypes) serializer uses: a 4-byte encapsulation header
     (0x00 0x01 0x00 0x00 = plain CDR, little-endian), then each field
     aligned to its natural size relative to the start of that header.

  2. The rosbag2_storage_sqlite3 on-disk layout: a schema/topics/messages
     SQLite3 database (schema_version 3, matching ROS 2 Humble) plus a
     hand-written metadata.yaml sidecar (no PyYAML dependency -- stdlib
     only, per this task's constraint).

Duration is ~152 s so it comfortably covers TS-06's 90 s (30 s before / 30 s
armed / 30 s after) window and TS-05's 60 s replay-at-rate cases.

Usage:
    python3 src/mapping/tests/fixtures/generate_f3_bag.py [--validate-only]

Writes tests/fixtures/f3_survey_replay/{metadata.yaml,f3_survey_replay_0.db3}
and then round-trips a sample of decoded messages against the values used to
generate them, per topic, printing PASS/FAIL for each check.
"""

from __future__ import annotations

import argparse
import os
import sqlite3
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BAG_DIR = os.path.join(HERE, "f3_survey_replay")
DB_NAME = "f3_survey_replay_0.db3"

# ---------------------------------------------------------------------------
# Scenario parameters (deterministic, matches TP-002 / TS-05 / TS-06 needs).
# ---------------------------------------------------------------------------

DURATION_S = 152.0
IMAGE_RATE_HZ = 15.0
POSE_RATE_HZ = 30.0
GNSS_RATE_HZ = 5.0

IMG_W = 4
IMG_H = 4
IMG_STEP = IMG_W * 3  # bgr8 == 3 bytes/px
IMG_DATA_LEN = IMG_STEP * IMG_H

SURVEY_SPEED_MS = 5.0     # matches test_survey_recorder_replay.py SURVEY_SPEED_MS
SURVEY_ALTITUDE_M = 40.0  # matches test_survey_recorder_replay.py SURVEY_ALTITUDE_M

BASE_LAT = 37.4275
BASE_LON = -122.1697

# Arbitrary but realistic epoch (2024-01-15T00:00:00Z-ish); only relative
# spacing matters for the replay driver.
BASE_NS = 1_705_276_800_000_000_000

IMAGE_TOPIC = "/oak/rgb/image_raw"
POSE_TOPIC = "/mavros/local_position/pose"
GNSS_TOPIC = "/mavros/global_position/global"

TOPICS = [
    (IMAGE_TOPIC, "sensor_msgs/msg/Image"),
    (POSE_TOPIC, "geometry_msgs/msg/PoseStamped"),
    (GNSS_TOPIC, "sensor_msgs/msg/NavSatFix"),
]

# ---------------------------------------------------------------------------
# CDR (plain, little-endian) writer / reader.
#
# Alignment is tracked relative to the start of the buffer *including* the
# 4-byte encapsulation header -- this matches the OMG CDR rule that the
# encapsulation header itself counts as the first 4 bytes of the aligned
# stream, so an 8-byte field immediately after the header needs 4 bytes of
# padding to reach an 8-aligned offset.
# ---------------------------------------------------------------------------

ENCAPSULATION_HEADER = bytes([0x00, 0x01, 0x00, 0x00])  # CDR_LE, no options


class CDRWriter:
    def __init__(self):
        self.buf = bytearray(ENCAPSULATION_HEADER)

    def _align(self, n: int):
        pad = (-len(self.buf)) % n
        if pad:
            self.buf += b"\x00" * pad

    def i8(self, v: int):
        self._align(1)
        self.buf += struct.pack("<b", v)

    def u8(self, v: int):
        self._align(1)
        self.buf += struct.pack("<B", v)

    def u16(self, v: int):
        self._align(2)
        self.buf += struct.pack("<H", v)

    def i32(self, v: int):
        self._align(4)
        self.buf += struct.pack("<i", v)

    def u32(self, v: int):
        self._align(4)
        self.buf += struct.pack("<I", v)

    def f64(self, v: float):
        self._align(8)
        self.buf += struct.pack("<d", v)

    def string(self, s: str):
        raw = s.encode("utf-8") + b"\x00"
        self._align(4)
        self.buf += struct.pack("<I", len(raw))
        self.buf += raw

    def u8_seq(self, data: bytes):
        self._align(4)
        self.buf += struct.pack("<I", len(data))
        self.buf += data  # uint8 elements: 1-aligned, no per-element padding

    def bytes(self) -> bytes:
        return bytes(self.buf)


class CDRReader:
    def __init__(self, data: bytes):
        self.buf = data
        self.pos = 0
        header = self._raw(4)
        if header != ENCAPSULATION_HEADER:
            raise ValueError(f"unexpected CDR encapsulation header: {header!r}")

    def _align(self, n: int):
        pad = (-self.pos) % n
        self.pos += pad

    def _raw(self, n: int) -> bytes:
        b = self.buf[self.pos:self.pos + n]
        self.pos += n
        return b

    def i8(self) -> int:
        self._align(1)
        return struct.unpack("<b", self._raw(1))[0]

    def u8(self) -> int:
        self._align(1)
        return struct.unpack("<B", self._raw(1))[0]

    def u16(self) -> int:
        self._align(2)
        return struct.unpack("<H", self._raw(2))[0]

    def i32(self) -> int:
        self._align(4)
        return struct.unpack("<i", self._raw(4))[0]

    def u32(self) -> int:
        self._align(4)
        return struct.unpack("<I", self._raw(4))[0]

    def f64(self) -> float:
        self._align(8)
        return struct.unpack("<d", self._raw(8))[0]

    def string(self) -> str:
        self._align(4)
        n = struct.unpack("<I", self._raw(4))[0]
        raw = self._raw(n)
        return raw[:-1].decode("utf-8")  # drop null terminator

    def u8_seq(self) -> bytes:
        self._align(4)
        n = struct.unpack("<I", self._raw(4))[0]
        return self._raw(n)


# ---------------------------------------------------------------------------
# Message builders / decoders.
# ---------------------------------------------------------------------------

def _write_header(w: CDRWriter, sec: int, nanosec: int, frame_id: str):
    w.i32(sec)
    w.u32(nanosec)
    w.string(frame_id)


def _read_header(r: CDRReader):
    sec = r.i32()
    nanosec = r.u32()
    frame_id = r.string()
    return sec, nanosec, frame_id


def encode_image(sec, nanosec, frame_id, height, width, encoding, is_bigendian, step, data: bytes) -> bytes:
    w = CDRWriter()
    _write_header(w, sec, nanosec, frame_id)
    w.u32(height)
    w.u32(width)
    w.string(encoding)
    w.u8(is_bigendian)
    w.u32(step)
    w.u8_seq(data)
    return w.bytes()


def decode_image(raw: bytes) -> dict:
    r = CDRReader(raw)
    sec, nanosec, frame_id = _read_header(r)
    height = r.u32()
    width = r.u32()
    encoding = r.string()
    is_bigendian = r.u8()
    step = r.u32()
    data = r.u8_seq()
    return dict(sec=sec, nanosec=nanosec, frame_id=frame_id, height=height, width=width,
                encoding=encoding, is_bigendian=is_bigendian, step=step, data=data)


def encode_pose_stamped(sec, nanosec, frame_id, x, y, z, qx, qy, qz, qw) -> bytes:
    w = CDRWriter()
    _write_header(w, sec, nanosec, frame_id)
    w.f64(x)
    w.f64(y)
    w.f64(z)
    w.f64(qx)
    w.f64(qy)
    w.f64(qz)
    w.f64(qw)
    return w.bytes()


def decode_pose_stamped(raw: bytes) -> dict:
    r = CDRReader(raw)
    sec, nanosec, frame_id = _read_header(r)
    x = r.f64(); y = r.f64(); z = r.f64()
    qx = r.f64(); qy = r.f64(); qz = r.f64(); qw = r.f64()
    return dict(sec=sec, nanosec=nanosec, frame_id=frame_id, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)


def encode_navsatfix(sec, nanosec, frame_id, status, service, latitude, longitude, altitude,
                      position_covariance, position_covariance_type) -> bytes:
    w = CDRWriter()
    _write_header(w, sec, nanosec, frame_id)
    w.i8(status)
    w.u16(service)
    w.f64(latitude)
    w.f64(longitude)
    w.f64(altitude)
    assert len(position_covariance) == 9
    for v in position_covariance:
        w.f64(v)
    w.u8(position_covariance_type)
    return w.bytes()


def decode_navsatfix(raw: bytes) -> dict:
    r = CDRReader(raw)
    sec, nanosec, frame_id = _read_header(r)
    status = r.i8()
    service = r.u16()
    latitude = r.f64()
    longitude = r.f64()
    altitude = r.f64()
    covariance = [r.f64() for _ in range(9)]
    covariance_type = r.u8()
    return dict(sec=sec, nanosec=nanosec, frame_id=frame_id, status=status, service=service,
                latitude=latitude, longitude=longitude, altitude=altitude,
                position_covariance=covariance, position_covariance_type=covariance_type)


# ---------------------------------------------------------------------------
# Scenario generation.
# ---------------------------------------------------------------------------

def _stamp(t_s: float):
    ns = BASE_NS + round(t_s * 1e9)
    return ns // 1_000_000_000, ns % 1_000_000_000, ns


def gen_image_messages():
    n = int(DURATION_S * IMAGE_RATE_HZ) + 1
    out = []
    for i in range(n):
        t = i / IMAGE_RATE_HZ
        sec, nanosec, ns = _stamp(t)
        data = bytes((i + k) % 256 for k in range(IMG_DATA_LEN))
        raw = encode_image(sec, nanosec, "oak_rgb_camera_frame", IMG_H, IMG_W, "bgr8", 0, IMG_STEP, data)
        out.append((ns, IMAGE_TOPIC, raw, dict(i=i, sec=sec, nanosec=nanosec, data=data)))
    return out


def gen_pose_messages():
    n = int(DURATION_S * POSE_RATE_HZ) + 1
    out = []
    for i in range(n):
        t = i / POSE_RATE_HZ
        sec, nanosec, ns = _stamp(t)
        x = SURVEY_SPEED_MS * t
        y = 0.0
        z = SURVEY_ALTITUDE_M
        raw = encode_pose_stamped(sec, nanosec, "base_link", x, y, z, 0.0, 0.0, 0.0, 1.0)
        out.append((ns, POSE_TOPIC, raw, dict(i=i, sec=sec, nanosec=nanosec, x=x, y=y, z=z)))
    return out


def gen_gnss_messages():
    n = int(DURATION_S * GNSS_RATE_HZ) + 1
    out = []
    for i in range(n):
        t = i / GNSS_RATE_HZ
        sec, nanosec, ns = _stamp(t)
        lat = BASE_LAT + 1e-5 * t
        lon = BASE_LON + 1e-5 * t
        alt = SURVEY_ALTITUDE_M
        cov = [0.0] * 9
        raw = encode_navsatfix(sec, nanosec, "gnss_link", 0, 1, lat, lon, alt, cov, 0)
        out.append((ns, GNSS_TOPIC, raw, dict(i=i, sec=sec, nanosec=nanosec, lat=lat, lon=lon, alt=alt)))
    return out


# ---------------------------------------------------------------------------
# rosbag2 sqlite3 storage.
# ---------------------------------------------------------------------------

def write_bag(messages):
    """messages: list of (timestamp_ns, topic, cdr_bytes, expected-dict), any order."""
    os.makedirs(BAG_DIR, exist_ok=True)
    db_path = os.path.join(BAG_DIR, DB_NAME)
    if os.path.exists(db_path):
        os.remove(db_path)

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL)")
    cur.execute("INSERT INTO schema (schema_version, ros_distro) VALUES (3, 'humble')")
    cur.execute(
        "CREATE TABLE topics("
        "id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
        "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)"
    )
    cur.execute(
        "CREATE TABLE messages("
        "id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, "
        "data BLOB NOT NULL)"
    )
    cur.execute("CREATE INDEX timestamp_idx ON messages (timestamp ASC)")

    topic_ids = {}
    for name, ty in TOPICS:
        cur.execute(
            "INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) "
            "VALUES (?, ?, 'cdr', '')",
            (name, ty),
        )
        topic_ids[name] = cur.lastrowid

    ordered = sorted(messages, key=lambda m: m[0])
    cur.executemany(
        "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
        [(topic_ids[topic], ts, sqlite3.Binary(raw)) for ts, topic, raw, _exp in ordered],
    )
    conn.commit()
    conn.close()
    return ordered, topic_ids


def write_metadata_yaml(ordered, per_topic_counts):
    start_ns = ordered[0][0]
    end_ns = ordered[-1][0]
    duration_ns = end_ns - start_ns
    message_count = len(ordered)

    lines = []
    lines.append("rosbag2_bagfile_information:")
    lines.append("  version: 5")
    lines.append("  storage_identifier: sqlite3")
    lines.append("  relative_file_paths:")
    lines.append(f"    - {DB_NAME}")
    lines.append("  duration:")
    lines.append(f"    nanoseconds: {duration_ns}")
    lines.append("  starting_time:")
    lines.append(f"    nanoseconds_since_epoch: {start_ns}")
    lines.append(f"  message_count: {message_count}")
    lines.append("  topics_with_message_count:")
    for name, ty in TOPICS:
        lines.append("    - topic_metadata:")
        lines.append(f"        name: {name}")
        lines.append(f"        type: {ty}")
        lines.append("        serialization_format: cdr")
        lines.append("        offered_qos_profiles: ''")
        lines.append(f"      message_count: {per_topic_counts[name]}")
    lines.append("  compression_format: ''")
    lines.append("  compression_mode: ''")
    text = "\n".join(lines) + "\n"

    with open(os.path.join(BAG_DIR, "metadata.yaml"), "w") as f:
        f.write(text)


# ---------------------------------------------------------------------------
# Round-trip validation (our own stdlib reader, independent of ROS 2).
# ---------------------------------------------------------------------------

def validate(expected_by_topic, n_samples=3):
    db_path = os.path.join(BAG_DIR, DB_NAME)
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    ok = True

    cur.execute("SELECT schema_version, ros_distro FROM schema")
    schema_version, ros_distro = cur.fetchone()
    print(f"[schema] schema_version={schema_version} ros_distro={ros_distro} "
          f"{'PASS' if schema_version == 3 else 'FAIL'}")
    ok &= schema_version == 3

    cur.execute("SELECT id, name, type, serialization_format FROM topics ORDER BY id")
    topic_rows = cur.fetchall()
    topic_by_id = {r[0]: r[1] for r in topic_rows}
    print(f"[topics] {len(topic_rows)} topics: "
          f"{[(r[1], r[2], r[3]) for r in topic_rows]}")
    ok &= len(topic_rows) == len(TOPICS)

    decoders = {
        IMAGE_TOPIC: decode_image,
        POSE_TOPIC: decode_pose_stamped,
        GNSS_TOPIC: decode_navsatfix,
    }

    for name, _ty in TOPICS:
        topic_id = next(r[0] for r in topic_rows if r[1] == name)
        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id = ?", (topic_id,))
        (count,) = cur.fetchone()
        expected_n = len(expected_by_topic[name])
        count_ok = count == expected_n
        ok &= count_ok
        print(f"[{name}] message_count={count} expected={expected_n} "
              f"{'PASS' if count_ok else 'FAIL'}")

        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topic_id,),
        )
        rows = cur.fetchall()

        # Monotonic timestamp check.
        timestamps = [r[0] for r in rows]
        mono_ok = all(timestamps[i] < timestamps[i + 1] for i in range(len(timestamps) - 1))
        ok &= mono_ok
        print(f"[{name}] timestamps strictly increasing: {'PASS' if mono_ok else 'FAIL'}")

        decode_fn = decoders[name]
        sample_idxs = [0, len(rows) // 2, len(rows) - 1][:min(n_samples, len(rows))]
        for idx in sample_idxs:
            ts, raw = rows[idx]
            decoded = decode_fn(bytes(raw))
            expected = expected_by_topic[name][idx]
            row_ok = (decoded["sec"] == expected["sec"] and decoded["nanosec"] == expected["nanosec"])
            if name == IMAGE_TOPIC:
                row_ok = row_ok and decoded["data"] == expected["data"] and decoded["width"] == IMG_W \
                    and decoded["height"] == IMG_H and decoded["encoding"] == "bgr8"
            elif name == POSE_TOPIC:
                row_ok = row_ok and abs(decoded["x"] - expected["x"]) < 1e-9 \
                    and abs(decoded["z"] - expected["z"]) < 1e-9
            elif name == GNSS_TOPIC:
                row_ok = row_ok and abs(decoded["latitude"] - expected["lat"]) < 1e-9 \
                    and abs(decoded["longitude"] - expected["lon"]) < 1e-9
            ok &= row_ok
            print(f"[{name}] sample idx={idx} ts={ts} decode round-trip: "
                  f"{'PASS' if row_ok else 'FAIL'}")

    conn.close()
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--validate-only", action="store_true",
                     help="skip regeneration, just validate the existing bag")
    args = ap.parse_args()

    image_msgs = gen_image_messages()
    pose_msgs = gen_pose_messages()
    gnss_msgs = gen_gnss_messages()

    expected_by_topic = {
        IMAGE_TOPIC: [m[3] for m in image_msgs],
        POSE_TOPIC: [m[3] for m in pose_msgs],
        GNSS_TOPIC: [m[3] for m in gnss_msgs],
    }
    per_topic_counts = {k: len(v) for k, v in expected_by_topic.items()}

    if not args.validate_only:
        all_messages = image_msgs + pose_msgs + gnss_msgs
        ordered, _topic_ids = write_bag(all_messages)
        write_metadata_yaml(ordered, per_topic_counts)
        print(f"wrote {os.path.join(BAG_DIR, DB_NAME)} "
              f"({len(all_messages)} messages, "
              f"{ordered[-1][0] - ordered[0][0]} ns span)")
        print(f"wrote {os.path.join(BAG_DIR, 'metadata.yaml')}")

    print("\n--- round-trip validation ---")
    ok = validate(expected_by_topic)
    print(f"\nOVERALL: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
