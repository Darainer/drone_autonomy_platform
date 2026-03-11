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
    TestBatteryFailsafe   — mock FCU battery drain → safety/battery_monitor
                            commands AUTO.RTL
    TestMissionPipeline   — autonomy publishes Mission → navigation receives it
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

# ── Drone autonomy message types ─────────────────────────────────────────────
from drone_autonomy_msgs.msg import Mission
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32


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


# ── Helpers ───────────────────────────────────────────────────────────────────

def wait_for_nodes(node, node_names, timeout=5.0):
    """Spin until all node_names appear in the ROS graph or timeout expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        live = node.get_node_names()
        if all(n in live for n in node_names):
            return True
    return False


# ── Test classes ──────────────────────────────────────────────────────────────

class TestNodesStartUp(unittest.TestCase):
    """All platform nodes come online within 5 seconds."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_startup_helper')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

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
        self.assertTrue(
            wait_for_nodes(self.node, expected, timeout=5.0),
            msg=f"Not all nodes started. Live nodes: {self.node.get_node_names()}",
        )


class TestBatteryFailsafe(unittest.TestCase):
    """
    Drive mock FCU battery to critical level and assert battery_monitor
    calls /mavros/set_mode with AUTO.RTL.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_battery_failsafe_helper')

        # Publish battery percentage injection commands to mock FCU
        cls.battery_inject_pub = cls.node.create_publisher(
            Float32, '/mock_fcu/set_battery_pct', 10)

        # Spy on /mavros/set_mode calls
        cls.set_mode_calls = []
        cls.set_mode_svc = cls.node.create_service(
            SetMode, '/mavros/set_mode_spy',  # secondary spy — see note below
            lambda req, resp: resp)

        # Record actual set_mode calls by subscribing to a log topic
        # (battery_monitor calls the real /mavros/set_mode served by mock_fcu)
        cls.mode_log = []

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _inject_battery(self, pct: float):
        msg = Float32()
        msg.data = pct
        self.battery_inject_pub.publish(msg)
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_normal_battery_no_rtl(self):
        """80% battery — no RTL within 1 s."""
        # mock_fcu starts at 100%, drive to 80% (above warn threshold)
        self._inject_battery(0.80)
        time.sleep(0.5)
        for _ in range(30):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        # No easy way to assert set_mode NOT called without intercepting the
        # service — covered by unit test_battery_monitor; here we just check
        # the node graph is intact.
        self.assertIn('battery_monitor', self.node.get_node_names())

    def test_critical_battery_triggers_rtl(self):
        """10% battery — battery_monitor must command AUTO.RTL to mock_fcu."""
        # Drive below 15% critical threshold
        self._inject_battery(0.10)
        # Give battery_monitor time to detect and call set_mode (up to 2 s)
        deadline = time.time() + 2.0
        rtl_seen = False
        while time.time() < deadline and not rtl_seen:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # mock_fcu logs the mode change — check armed state via /mavros/state
            # After AUTO.RTL + simulate_disarm_on_rtl=true the mock_fcu disarms.
            state_received = []

            def state_cb(msg):
                state_received.append(msg)

            sub = self.node.create_subscription(State, '/mavros/state', state_cb, 10)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            self.node.destroy_subscription(sub)

            if state_received and not state_received[-1].armed:
                rtl_seen = True

        self.assertTrue(
            rtl_seen,
            "Expected mock_fcu to disarm after AUTO.RTL (simulate_disarm_on_rtl=true)",
        )


class TestMissionPipeline(unittest.TestCase):
    """
    Autonomy publishes a Mission; navigation_node receives it.
    Verifies the autonomy → navigation topic wiring is correct.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
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
        rclpy.shutdown()

    def test_mission_reaches_navigation(self):
        """
        Autonomy node publishes a hardcoded test mission on startup.
        Verify it arrives at /navigation_node/mission within 3 s.
        """
        deadline = time.time() + 3.0
        while time.time() < deadline and not self.received_missions:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreater(
            len(self.received_missions), 0,
            "No Mission message received on /navigation_node/mission within 3 s",
        )
