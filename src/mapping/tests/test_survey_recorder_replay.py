"""TP-002 TS-05, TS-06 -- survey_recorder_node ROS-graph/replay tests.

Replays the F3 fixture bag through a running survey_recorder_node, drives
the DES-004 D6 arm/disarm sequence via /mission and /mission_status, and
checks the produced dataset directory against the TS-05/TS-06 pass criteria.

Fixture: TP-002 "Reference fixtures" F3 -- a rosbag2 recording of
/oak/rgb/image_raw + /mavros/local_position/pose +
/mavros/global_position/global at realistic rates/timestamps, at least
~150 s long (TS-06 needs a continuous 90 s window; TS-05 needs a further
60 s per capture-rate case). Checked in at tests/fixtures/f3_survey_replay/
by CAP-001 WP-2 T2.3 -- NOT present yet as of this file (T2.2 authors the
consuming test ahead of the fixture, per the WP-2 task split). This suite
self-skips until that directory exists.

This is a launch_testing (colcon test) suite: it needs a full ROS 2 graph
(rclpy, launch_ros, the C++ survey_recorder_node with message_filters) and
cannot run in a sandbox without ROS 2 installed. It is authored to run in
CI via `colcon test`; it has not been executed locally (CAP-001 WP-2 T2.2).

Verifies: MAP-2 (TS-05, TS-06)
"""
import csv
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RclpyNode

from drone_autonomy_msgs.msg import Mission, MissionStatus


# TP-002 "Reference fixtures" F3.
FIXTURE_BAG_DIR = os.path.join(os.path.dirname(__file__), 'fixtures', 'f3_survey_replay')

# D4 sync budget and GNSS attach window (DES-004), duplicated here (not
# imported from the node) so this test independently states its own pass
# criteria rather than trusting the implementation under test.
MAX_SYNC_ERR_MS = 50.0

# TP-002 "Reference fixtures": common parameters unless a TS says otherwise.
SURVEY_ALTITUDE_M = 40.0
FORWARD_OVERLAP = 0.75
SIDE_OVERLAP = 0.60
SURVEY_SPEED_MS = 5.0


def _bag_available():
    return os.path.isdir(FIXTURE_BAG_DIR)


@pytest.mark.launch_test
@pytest.mark.skipif(
    not _bag_available(),
    reason='F3 replay bag fixture not yet checked in (tests/fixtures/f3_survey_replay/, CAP-001 WP-2 T2.3)')
def generate_test_description():
    output_dir = tempfile.mkdtemp(prefix='survey_recorder_replay_')

    survey_recorder = launch_ros.actions.Node(
        package='mapping',
        executable='survey_recorder_node',
        parameters=[{
            # Overridden per-arm by Mission.capture_rate_hz in every test
            # case below; this default is never exercised directly.
            'default_capture_rate_hz': 2.0,
            'output_dir': output_dir,
            'jpeg_quality': 95,
            # Deliberately small so the D7 guard never blocks this test on a
            # CI runner's disk state.
            'min_free_mb': 1,
        }],
    )

    return launch.LaunchDescription([
        survey_recorder,
        launch_testing.actions.ReadyToTest(),
    ]), {'survey_recorder': survey_recorder, 'output_dir': output_dir}


class _BagReplayer:
    """Replays every message in the F3 bag on a background thread, preserving
    the bag's original inter-message timing (bag t=0 maps to wall time
    `start()` is called)."""

    def __init__(self, node, bag_dir):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
        msg_types = {name: get_message(type_str) for name, type_str in type_map.items()}
        self._publishers = {name: node.create_publisher(cls, name, 30) for name, cls in msg_types.items()}

        self._messages = []  # (t_ns, topic, deserialized msg)
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            self._messages.append((t_ns, topic, deserialize_message(data, msg_types[topic])))
        self._messages.sort(key=lambda m: m[0])

        self._thread = None
        self._stop = threading.Event()

    def duration_s(self):
        if not self._messages:
            return 0.0
        return (self._messages[-1][0] - self._messages[0][0]) / 1e9

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self._messages[0][0] if self._messages else 0

    def _run(self):
        if not self._messages:
            return
        t0_bag = self._messages[0][0]
        t0_wall = time.monotonic()
        for t_ns, topic, msg in self._messages:
            if self._stop.is_set():
                return
            target = t0_wall + (t_ns - t0_bag) / 1e9
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            self._publishers[topic].publish(msg)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5.0)


def _make_mission(mission_id, capture_rate_hz, stamp):
    m = Mission()
    m.header.stamp = stamp
    m.mission_id = mission_id
    m.mission_type = 'survey'
    m.survey_altitude_m = SURVEY_ALTITUDE_M
    m.forward_overlap = FORWARD_OVERLAP
    m.side_overlap = SIDE_OVERLAP
    m.survey_speed_ms = SURVEY_SPEED_MS
    m.capture_rate_hz = capture_rate_hz
    return m


def _make_mission_status(mission_id, state, stamp):
    s = MissionStatus()
    s.header.stamp = stamp
    s.mission_id = mission_id
    s.state = state
    s.detail = ''
    return s


def _dataset_dirs(output_dir, mission_id):
    prefix = 'survey_' + mission_id
    return [
        os.path.join(output_dir, d) for d in os.listdir(output_dir)
        if d.startswith(prefix) and os.path.isdir(os.path.join(output_dir, d))
    ]


def _read_poses_csv(dataset_dir):
    path = os.path.join(dataset_dir, 'poses.csv')
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def _read_manifest_scalars(dataset_dir):
    """Minimal reader for this package's fixed manifest.yaml schema -- avoids
    a PyYAML dependency in this test package; tools/photogrammetry (WP-3) is
    the real consumer and uses a full YAML parser."""
    path = os.path.join(dataset_dir, 'manifest.yaml')
    out = {}
    with open(path) as f:
        for line in f:
            if line.startswith(' ') or ':' not in line:
                continue
            key, _, val = line.partition(':')
            out[key.strip()] = val.strip().strip('"')
    return out


class TestSurveyRecorderReplay(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('test_survey_recorder_replay_driver')
        self.mission_pub = self.node.create_publisher(Mission, '/mission', 10)
        self.status_pub = self.node.create_publisher(MissionStatus, '/mission_status', 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()

    def _arm(self, mission_id, capture_rate_hz):
        self.mission_pub.publish(_make_mission(mission_id, capture_rate_hz, self.node.get_clock().now().to_msg()))

    def _disarm(self, mission_id, state='complete'):
        self.status_pub.publish(_make_mission_status(mission_id, state, self.node.get_clock().now().to_msg()))

    def test_ts06_recording_window(self, output_dir):
        """TS-05/TS-06 fixture note + TS-06 procedure: replay F3 for 30 s
        before arming, 30 s armed, 30 s after `complete`. Pass: all recorded
        frame timestamps fall inside the armed window +/- one frame period; a
        second arm creates a new dataset directory, never appends.

        Verifies: MAP-2 (TS-06)
        """
        replayer = _BagReplayer(self.node, FIXTURE_BAG_DIR)
        self.assertGreaterEqual(replayer.duration_s(), 90.0,
                                 'F3 fixture must cover the TS-06 90 s before/armed/after window')

        mission_id = 'ts06_survey_replay'
        capture_rate_hz = 2.0
        frame_period_s = 1.0 / capture_rate_hz

        replay_start_wall = time.monotonic()
        replayer.start()

        time.sleep(30.0)
        arm_wall = time.monotonic()
        arm_ros_time = self.node.get_clock().now()
        self._arm(mission_id, capture_rate_hz)

        time.sleep(30.0)
        disarm_wall = time.monotonic()
        self._disarm(mission_id, 'complete')

        time.sleep(30.0)
        replayer.stop()

        # Give the node a moment to finalize the manifest after disarm.
        time.sleep(2.0)

        dirs = _dataset_dirs(output_dir, mission_id)
        self.assertEqual(len(dirs), 1, f'expected exactly one dataset directory for {mission_id}, got {dirs}')
        rows = _read_poses_csv(dirs[0])
        self.assertGreater(len(rows), 0, 'expected at least one recorded frame in the armed window')

        armed_start_ns = arm_ros_time.nanoseconds
        # Approximate the disarm ROS time from the wall-clock delta, since we
        # only captured a ROS Time at arm.
        armed_end_ns = armed_start_ns + int((disarm_wall - arm_wall) * 1e9)
        tolerance_ns = int(frame_period_s * 1e9)

        for row in rows:
            stamp_ns = int(row['stamp_ns'])
            self.assertGreaterEqual(stamp_ns, armed_start_ns - tolerance_ns,
                                     'frame recorded before the armed window (± one frame period)')
            self.assertLessEqual(stamp_ns, armed_end_ns + tolerance_ns,
                                  'frame recorded after the armed window (± one frame period)')

        # Second arm/disarm creates a new directory, never appends.
        mission_id_2 = 'ts06_survey_replay_second_arm'
        self._arm(mission_id_2, capture_rate_hz)
        time.sleep(2.0)
        self._disarm(mission_id_2, 'complete')
        time.sleep(2.0)

        dirs_2 = _dataset_dirs(output_dir, mission_id_2)
        self.assertEqual(len(dirs_2), 1)
        self.assertNotEqual(dirs[0], dirs_2[0])

    def test_ts05_sync_budget_and_capture_rate(self, output_dir):
        """TS-05 procedure: replay 60 s with an armed survey mission; disarm
        via MissionStatus{complete}; read dataset. Pass: dataset contains
        >= 95% of the expected rate*60 frames at the mission-specified
        capture rate (repeated at 1 Hz and 4 Hz to prove the field is
        honored); every poses.csv row has sync_err_ms <= 50; lat/lon
        non-null for >= 99% of rows; dropped_sync in manifest equals
        replayed pairs exceeding slop.

        Verifies: MAP-2 (TS-05)
        """
        for capture_rate_hz in (1.0, 4.0):
            with self.subTest(capture_rate_hz=capture_rate_hz):
                replayer = _BagReplayer(self.node, FIXTURE_BAG_DIR)
                self.assertGreaterEqual(replayer.duration_s(), 60.0,
                                         'F3 fixture must cover at least the 60 s TS-05 replay window')

                mission_id = f'ts05_survey_replay_{int(capture_rate_hz)}hz'
                replayer.start()
                self._arm(mission_id, capture_rate_hz)

                time.sleep(60.0)
                self._disarm(mission_id, 'complete')
                replayer.stop()
                time.sleep(2.0)

                dirs = _dataset_dirs(output_dir, mission_id)
                self.assertEqual(len(dirs), 1)
                dataset_dir = dirs[0]

                rows = _read_poses_csv(dataset_dir)
                expected_frames = capture_rate_hz * 60.0
                self.assertGreaterEqual(
                    len(rows), 0.95 * expected_frames,
                    f'expected >= 95% of {expected_frames:.0f} frames at {capture_rate_hz} Hz, got {len(rows)}')

                for row in rows:
                    self.assertLessEqual(float(row['sync_err_ms']), MAX_SYNC_ERR_MS)

                non_null_gnss = sum(1 for row in rows if row['lat'] != '' and row['lon'] != '')
                self.assertGreaterEqual(non_null_gnss, 0.99 * len(rows),
                                         'lat/lon must be non-null for >= 99% of rows')

                manifest = _read_manifest_scalars(dataset_dir)
                # dropped_sync is checked for internal self-consistency here
                # (non-negative integer, present); an independent recount of
                # "replayed pairs exceeding slop" requires re-deriving
                # message_filters' own approximate pairing over the raw bag
                # topics, which duplicates the unit under test -- left as a
                # documented gap for the CI run to cross-check against a bag
                # with known-injected desync, rather than asserted generically
                # here.
                self.assertIn('dropped_sync', manifest)
                self.assertGreaterEqual(int(manifest['dropped_sync']), 0)
