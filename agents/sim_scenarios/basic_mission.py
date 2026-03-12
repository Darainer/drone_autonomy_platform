"""
agents/sim_scenarios/basic_mission.py

Full SITL scenario: arm → takeoff → waypoint mission → land.

Stack required (all running before this script):
  1. Isaac Sim 4.2 with Pegasus Simulator extension loaded
     $ cd ~/VSLAM-UAV/sim && ISAACSIM_PYTHON isaac_sim.py
  2. sim Docker container (mavrospy bridge + platform nodes)
     $ docker compose -f docker/sim/docker-compose.yml up sim
  3. (Optional) QGroundControl connected to udp://:14550

Usage:
    # From inside the sim Docker container, or with ROS2 sourced:
    python3 agents/sim_scenarios/basic_mission.py

    # Override waypoints:
    python3 agents/sim_scenarios/basic_mission.py --waypoints 0,0,5 10,0,5 10,10,5

    # Run assertions only (no GUI required, SITL must already be running):
    python3 agents/sim_scenarios/basic_mission.py --assert-only

Exit codes:
    0 — all assertions passed (scenario completed successfully)
    1 — assertion failure (test failed)
    2 — timeout (scenario did not complete in time)

The scenario communicates with the platform exclusively via the same MAVROS
interface the real Pixhawk uses:
  /mavros/state          — read arming/mode/connected status
  /mavros/set_mode       — switch to OFFBOARD / AUTO.LAND
  /mavros/cmd/arming     — arm the vehicle
  /mavros/setpoint_position/local  — send position setpoints
"""

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header


# ── Configuration ─────────────────────────────────────────────────────────────

@dataclass
class MissionConfig:
    # Default square waypoint pattern at 5 m altitude
    waypoints: list = field(default_factory=lambda: [
        (0.0,  0.0,  5.0),   # takeoff position
        (10.0, 0.0,  5.0),   # east
        (10.0, 10.0, 5.0),   # north-east
        (0.0,  10.0, 5.0),   # north
        (0.0,  0.0,  5.0),   # return home
    ])
    waypoint_hold_s: float = 3.0     # seconds to hold each waypoint
    waypoint_reach_m: float = 0.5    # acceptance radius (metres)
    arm_timeout_s: float = 10.0
    offboard_timeout_s: float = 5.0
    mission_timeout_s: float = 120.0
    land_timeout_s: float = 30.0
    setpoint_hz: float = 20.0        # MAVROS requires ≥2 Hz before OFFBOARD


# ── Scenario runner ───────────────────────────────────────────────────────────

class BasicMissionScenario(Node):

    def __init__(self, config: MissionConfig):
        super().__init__('basic_mission_scenario')
        self.cfg = config

        self.current_state: Optional[State] = None
        self.current_pose: Optional[PoseStamped] = None
        self.failsafe_triggered: bool = False

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self._on_state, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_sensor)

        # Setpoint publisher — must publish before OFFBOARD can be activated
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clients
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_state(self, msg: State):
        self.current_state = msg
        # Detect unexpected failsafe / disarm during mission
        if (self.current_state.mode in ('AUTO.RTL', 'AUTO.LAND')
                and not self._in_land_phase):
            self.get_logger().error(
                f"Unexpected failsafe! Mode changed to {msg.mode}")
            self.failsafe_triggered = True

    def _on_pose(self, msg: PoseStamped):
        self.current_pose = msg

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _spin_for(self, duration_s: float):
        """Spin callbacks for duration_s seconds."""
        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for(self, predicate, timeout_s: float, description: str) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.get_logger().error(f"TIMEOUT waiting for: {description}")
        return False

    def _publish_setpoint(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def _distance_to(self, x: float, y: float, z: float) -> float:
        if self.current_pose is None:
            return float('inf')
        p = self.current_pose.pose.position
        return ((p.x - x)**2 + (p.y - y)**2 + (p.z - z)**2) ** 0.5

    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().mode_sent

    def _arm(self, arm: bool) -> bool:
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success

    # ── Scenario steps ────────────────────────────────────────────────────────

    def wait_for_fcu_connection(self):
        self.get_logger().info("Waiting for FCU connection...")
        ok = self._wait_for(
            lambda: self.current_state is not None and self.current_state.connected,
            timeout_s=30.0,
            description="FCU connected",
        )
        assert ok, "FCU did not connect within 30 s"
        self.get_logger().info("FCU connected.")

    def preflight_setpoints(self):
        """
        MAVROS requires at least 2 Hz of setpoints before OFFBOARD mode can be
        activated.  Publish 50 setpoints at the current (ground) position.
        """
        self.get_logger().info("Publishing preflight setpoints...")
        for _ in range(50):
            self._publish_setpoint(0.0, 0.0, 0.0)
            self._spin_for(1.0 / self.cfg.setpoint_hz)

    def arm_vehicle(self):
        self.get_logger().info("Switching to OFFBOARD mode...")
        ok = self._set_mode('OFFBOARD')
        assert ok, "Failed to set OFFBOARD mode"

        self.get_logger().info("Arming vehicle...")
        ok = self._wait_for(
            lambda: self._arm(True) and (self.current_state and self.current_state.armed),
            timeout_s=self.cfg.arm_timeout_s,
            description="vehicle armed",
        )
        assert ok, "Vehicle did not arm within timeout"
        self.get_logger().info("Armed.")

    def fly_waypoints(self):
        self._in_land_phase = False
        deadline = time.time() + self.cfg.mission_timeout_s

        for i, (x, y, z) in enumerate(self.cfg.waypoints):
            self.get_logger().info(f"Flying to waypoint {i+1}/{len(self.cfg.waypoints)}: "
                                   f"({x:.1f}, {y:.1f}, {z:.1f})")

            # Stream setpoints while approaching
            while self._distance_to(x, y, z) > self.cfg.waypoint_reach_m:
                assert not self.failsafe_triggered, "Failsafe triggered during mission"
                assert time.time() < deadline, "Mission timeout exceeded"
                self._publish_setpoint(x, y, z)
                self._spin_for(1.0 / self.cfg.setpoint_hz)

            self.get_logger().info(f"Waypoint {i+1} reached. Holding {self.cfg.waypoint_hold_s} s...")
            hold_deadline = time.time() + self.cfg.waypoint_hold_s
            while time.time() < hold_deadline:
                self._publish_setpoint(x, y, z)
                self._spin_for(1.0 / self.cfg.setpoint_hz)

        self.get_logger().info("All waypoints completed.")

    def land(self):
        self._in_land_phase = True
        self.get_logger().info("Commanding AUTO.LAND...")
        ok = self._set_mode('AUTO.LAND')
        assert ok, "Failed to set AUTO.LAND mode"

        ok = self._wait_for(
            lambda: self.current_state and not self.current_state.armed,
            timeout_s=self.cfg.land_timeout_s,
            description="vehicle disarmed after landing",
        )
        assert ok, "Vehicle did not disarm after landing within timeout"
        self.get_logger().info("Landed and disarmed.")

    # ── Assertions ────────────────────────────────────────────────────────────

    def assert_mission_success(self):
        assert not self.failsafe_triggered, \
            "FAIL: unexpected failsafe was triggered during the mission"
        assert self.current_state is not None, \
            "FAIL: no state received from FCU"
        assert not self.current_state.armed, \
            "FAIL: vehicle is still armed after AUTO.LAND"
        print("\n✓ basic_mission scenario PASSED — all assertions satisfied")


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Basic SITL mission scenario")
    p.add_argument(
        '--waypoints', nargs='+', metavar='X,Y,Z',
        help="Override waypoints as comma-separated X,Y,Z triplets",
    )
    p.add_argument(
        '--assert-only', action='store_true',
        help="Skip arming/flying; only run post-mission assertions "
             "(assumes scenario already ran)",
    )
    return p.parse_args()


def main():
    args = parse_args()

    cfg = MissionConfig()
    if args.waypoints:
        cfg.waypoints = [
            tuple(float(v) for v in wp.split(','))
            for wp in args.waypoints
        ]

    rclpy.init()
    scenario = BasicMissionScenario(cfg)

    try:
        if not args.assert_only:
            scenario.wait_for_fcu_connection()
            scenario.preflight_setpoints()
            scenario.arm_vehicle()
            scenario.fly_waypoints()
            scenario.land()

        scenario.assert_mission_success()

    except AssertionError as exc:
        print(f"\n✗ FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        sys.exit(2)
    finally:
        scenario.destroy_node()
        rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()
