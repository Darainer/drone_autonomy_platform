"""
test_platform_integration.py

launch_testing integration suite for the drone autonomy platform.
Replaces the real MAVROS stack with mock_fcu so tests run without
hardware or PX4 SITL.

Run:
    colcon test --packages-select drone_autonomy_platform
    # or directly:
    python -m pytest launch/test/test_platform_integration.py -v

Test cases:
    TestNodesStartUp      — all core nodes come online within 5 s
    TestBatteryFailsafe   — mock FCU battery drain → battery_monitor
                            commands AUTO.RTL (verified via armed→false
                            transition on mock FCU)
    TestMissionPipeline   — autonomy publishes Mission → navigation receives it
    TestMockFCUInterface  — injection topics and published fields are correct

rclpy lifecycle note:
    rclpy.init()/shutdown() must be called exactly once per process.
    All test classes share a single rclpy context via setUpModule /
    tearDownModule; individual test classes only create/destroy nodes.
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from drone_autonomy_msgs.msg import Mission
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32


# ── Module-level rclpy lifecycle ─────────────────────────────────────────────
# rclpy must be initialized once for the entire test module.  All test classes
# share this context rather than calling init/shutdown themselves.

def setUpModule():    # noqa: N802  (camelCase required by unittest)
    rclpy.init()


def tearDownModule():  # noqa: N802
    rclpy.shutdown()


# ── Launch description ────────────────────────────────────────────────────────

@pytest.mark.launch_test
def generate_test_description():
    mock_fcu_share = get_package_share_directory('mock_fcu')

    mock_fcu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mock_fcu_share, 'launch', 'mock_fcu.launch.py')
        ),
        launch_arguments={
            'battery_percentage': '1.0',
            'armed': 'true',
            'publish_hz': '20.0',
            'simulate_disarm_on_rtl': 'true',
        }.items(),
    )

    autonomy_node = launch_ros.actions.Node(
        package='autonomy', executable='autonomy_node', name='autonomy_node')
    navigation_node = launch_ros.actions.Node(
        package='navigation', executable='navigation_node', name='navigation_node')
    control_node = launch_ros.actions.Node(
        package='control', executable='control_node', name='control_node')
    safety_node = launch_ros.actions.Node(
        package='safety', executable='safety_node', name='safety_node')
    battery_monitor = launch_ros.actions.Node(
        package='safety', executable='battery_monitor', name='battery_monitor')
    communication_node = launch_ros.actions.Node(
        package='communication', executable='communication_node',
        name='communication_node')

    return launch.LaunchDescription([
        mock_fcu,
        autonomy_node,
        navigation_node,
        control_node,
        safety_node,
        battery_monitor,
        communication_node,
        launch_testing.actions.ReadyToTest(),
    ])


# ── Shared helpers ────────────────────────────────────────────────────────────

def spin_for(node, duration_s: float, step_s: float = 0.05):
    deadline = time.time() + duration_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=step_s)


def wait_for_nodes(node, node_names: list, timeout: float = 5.0) -> bool:
    """Return True once all node_names appear in the ROS graph."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(n in node.get_node_names() for n in node_names):
            return True
    return False


def wait_for_condition(node, predicate, timeout: float = 3.0, step: float = 0.05) -> bool:
    """Spin until predicate() returns True or timeout expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=step)
        if predicate():
            return True
    return False


# ── Test classes ──────────────────────────────────────────────────────────────

class TestNodesStartUp(unittest.TestCase):
    """All platform nodes come online within 5 seconds."""

    @classmethod
    def setUpClass(cls):
        cls.node = rclpy.create_node('test_startup_helper')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()

    def test_all_nodes_running(self):
        expected = [
            'mock_fcu',
            'autonomy_node',
            'navigation_node',
            'control_node',
            'safety_node',
            'battery_monitor',
            'communication_node',
        ]
        ok = wait_for_nodes(self.node, expected, timeout=5.0)
        self.assertTrue(
            ok,
            msg=f"Not all nodes started. Live: {self.node.get_node_names()}",
        )


class TestBatteryFailsafe(unittest.TestCase):
    """
    Drive mock FCU battery to critical level via the /mock_fcu/set_battery_pct
    injection topic and assert battery_monitor triggers AUTO.RTL.

    Verification strategy: mock_fcu has simulate_disarm_on_rtl=true, so after
    AUTO.RTL is sent to /mavros/set_mode, mock_fcu transitions armed→false and
    sets mode to AUTO.RTL.  We subscribe to /mavros/state and watch for both.
    """

    @classmethod
    def setUpClass(cls):
        cls.node = rclpy.create_node('test_battery_failsafe_helper')

        cls.battery_inject = cls.node.create_publisher(
            Float32, '/mock_fcu/set_battery_pct', 10)

        cls.state_msgs = []
        cls.state_sub = cls.node.create_subscription(
            State, '/mavros/state',
            lambda msg: cls.state_msgs.append(msg),
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()

    def _set_battery(self, pct: float):
        msg = Float32()
        msg.data = pct
        self.battery_inject.publish(msg)

    def test_normal_battery_keeps_armed(self):
        """80% battery — mock FCU stays armed (well above 15% threshold)."""
        self.state_msgs.clear()
        self._set_battery(0.80)
        spin_for(self.node, 1.0)

        armed_states = [m.armed for m in self.state_msgs]
        if armed_states:
            self.assertTrue(
                armed_states[-1],
                "Expected drone to remain armed at 80% battery",
            )

    def test_critical_battery_triggers_disarm_via_rtl(self):
        """
        10% battery → battery_monitor calls AUTO.RTL → mock_fcu disarms.
        Wait up to 3 s for armed=false on /mavros/state.
        """
        self.state_msgs.clear()
        self._set_battery(0.10)

        disarmed = wait_for_condition(
            self.node,
            lambda: any(not m.armed for m in self.state_msgs),
            timeout=3.0,
        )
        self.assertTrue(
            disarmed,
            "Expected mock_fcu to disarm after battery_monitor sends AUTO.RTL "
            "(simulate_disarm_on_rtl=true). Recent states: "
            f"{[(m.armed, m.mode) for m in self.state_msgs[-5:]]}",
        )

    def test_rtl_mode_set_on_fcu(self):
        """
        Verify /mavros/state.mode becomes AUTO.RTL after critical battery
        injection (mock_fcu records and reflects the set_mode call).
        """
        self.state_msgs.clear()
        self._set_battery(0.10)

        rtl_seen = wait_for_condition(
            self.node,
            lambda: any(m.mode == 'AUTO.RTL' for m in self.state_msgs),
            timeout=3.0,
        )
        self.assertTrue(
            rtl_seen,
            "Expected /mavros/state.mode == 'AUTO.RTL' after critical battery. "
            f"Modes seen: {[m.mode for m in self.state_msgs[-5:]]}",
        )


class TestMissionPipeline(unittest.TestCase):
    """
    Autonomy publishes a Mission; navigation_node receives it.
    Verifies the autonomy → navigation topic wiring end-to-end.
    """

    @classmethod
    def setUpClass(cls):
        cls.node = rclpy.create_node('test_mission_pipeline_helper')
        cls.received_missions = []
        cls.mission_sub = cls.node.create_subscription(
            Mission,
            '/navigation_node/mission',
            lambda msg: cls.received_missions.append(msg),
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()

    def test_mission_reaches_navigation(self):
        """
        autonomy_node publishes a hardcoded test mission on startup.
        Verify it arrives at /navigation_node/mission within 3 s.
        """
        ok = wait_for_condition(
            self.node,
            lambda: len(self.received_missions) > 0,
            timeout=3.0,
        )
        self.assertTrue(
            ok,
            "No Mission received on /navigation_node/mission within 3 s",
        )

    def test_mission_has_nonempty_id(self):
        """The published mission must have a non-empty mission_id."""
        wait_for_condition(
            self.node,
            lambda: len(self.received_missions) > 0,
            timeout=3.0,
        )
        self.assertGreater(len(self.received_missions), 0)
        self.assertNotEqual(
            self.received_missions[0].mission_id, '',
            "mission_id should not be empty",
        )


class TestMockFCUInterface(unittest.TestCase):
    """
    Verify mock_fcu correctly publishes the MAVROS topics that all platform
    nodes subscribe to, and that injection commands work as expected.
    """

    @classmethod
    def setUpClass(cls):
        cls.node = rclpy.create_node('test_mock_fcu_helper')

        cls.state_msgs = []
        cls.battery_msgs = []

        cls.state_sub = cls.node.create_subscription(
            State, '/mavros/state',
            lambda m: cls.state_msgs.append(m), 10)

        cls.battery_sub = cls.node.create_subscription(
            BatteryState, '/mavros/battery',
            lambda m: cls.battery_msgs.append(m), 10)

        cls.armed_inject   = cls.node.create_publisher(Bool,    '/mock_fcu/set_armed',       10)
        cls.battery_inject = cls.node.create_publisher(Float32, '/mock_fcu/set_battery_pct', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()

    def test_state_topic_published(self):
        """mock_fcu publishes /mavros/state at startup."""
        ok = wait_for_condition(
            self.node, lambda: len(self.state_msgs) > 0, timeout=3.0)
        self.assertTrue(ok, "/mavros/state not received within 3 s")

    def test_battery_topic_published(self):
        """mock_fcu publishes /mavros/battery at startup."""
        ok = wait_for_condition(
            self.node, lambda: len(self.battery_msgs) > 0, timeout=3.0)
        self.assertTrue(ok, "/mavros/battery not received within 3 s")

    def test_battery_percentage_injection(self):
        """Injecting 50% via /mock_fcu/set_battery_pct is reflected on /mavros/battery."""
        target = 0.50
        msg = Float32()
        msg.data = target
        self.battery_inject.publish(msg)

        ok = wait_for_condition(
            self.node,
            lambda: any(abs(m.percentage - target) < 0.01 for m in self.battery_msgs),
            timeout=2.0,
        )
        self.assertTrue(ok, f"Expected battery ≈{target*100:.0f}% on /mavros/battery")

    def test_armed_injection(self):
        """Injecting armed=True via /mock_fcu/set_armed is reflected on /mavros/state."""
        self.state_msgs.clear()
        msg = Bool()
        msg.data = True
        self.armed_inject.publish(msg)

        ok = wait_for_condition(
            self.node,
            lambda: any(m.armed for m in self.state_msgs),
            timeout=2.0,
        )
        self.assertTrue(ok, "Expected armed=True on /mavros/state after injection")

    def test_state_connected_field(self):
        """mock_fcu always reports connected=True."""
        ok = wait_for_condition(
            self.node,
            lambda: any(m.connected for m in self.state_msgs),
            timeout=2.0,
        )
        self.assertTrue(ok, "Expected connected=True on /mavros/state")

    def test_battery_voltage_plausible(self):
        """mock_fcu computes voltage from percentage (4S LiPo: pct * 16.8 V)."""
        ok = wait_for_condition(
            self.node, lambda: len(self.battery_msgs) > 0, timeout=2.0)
        self.assertTrue(ok)
        for msg in self.battery_msgs[-3:]:
            expected_v = msg.percentage * 16.8
            self.assertAlmostEqual(
                msg.voltage, expected_v, places=1,
                msg=(f"Voltage {msg.voltage:.2f} V not consistent with "
                     f"{msg.percentage*100:.0f}% (expected {expected_v:.2f} V)"),
            )
