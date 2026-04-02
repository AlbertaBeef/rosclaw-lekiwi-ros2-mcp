from __future__ import annotations

"""
ROSClaw LeKiwi ROS2 MCP Server

LeKiwi Mobile Base + SO-101 Arm MCP Server using ROS2.
Part of the ROSClaw Embodied Intelligence Operating System.

Features:
- Omnidirectional mobile base control (3-wheel omni-drive)
- SO-101 arm control (imported from rosclaw-so101-ros2-mcp)
- Combined base+arm state monitoring
- Safety guards for base velocity and arm joint limits

Hardware: LeKiwi (3x STS3215 wheels, motor IDs 7-9) + SO-101 arm (motor IDs 1-6)
Protocol: ROS2 (Robot Operating System 2)

ROS2 Topics:
- /cmd_vel (geometry_msgs/Twist) — base velocity commands
- /joint_states (sensor_msgs/JointState) — all joint feedback (arm + wheels)
- /joint_commands (sensor_msgs/JointState) — arm position commands

Prerequisites:
    ros2 run lekiwi_hw_interface lekiwi_motor_bridge --ros-args \\
      -p port:=/dev/ttyACM0 -p use_so101:=true
"""

import asyncio
import logging
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

logger = logging.getLogger(__name__)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from mcp.server.fastmcp import FastMCP

# Import SO-101 arm components
try:
    from so101_mcp_server import SO101ROS2Bridge, SO101State
except ImportError:
    from src.so101_mcp_server import SO101ROS2Bridge, SO101State

# Initialize MCP Server
mcp = FastMCP("rosclaw-lekiwi")


# ============ Omni-Base Kinematics ============
# Adapted from Ekumen-OS/lekiwi LeKiwiMobileBase


class OmniBaseKinematics:
    """Forward and inverse kinematics for the LeKiwi 3-wheel omnidirectional base.

    Wheel layout (viewed from above):
    - Wheel 1 (left):  30 deg from forward (motor ID 7)
    - Wheel 2 (right): clockwise 120 deg from wheel 1 (motor ID 9)
    - Wheel 3 (back):  clockwise 240 deg from wheel 1 (motor ID 8)

    Conventions:
    - Forward: +x, Left: +y, CCW rotation: +omega
    """

    def __init__(
        self,
        wheel_radius: float = 0.054,
        base_radius: float = 0.125,
    ):
        self.wheel_radius = wheel_radius
        self.base_radius = base_radius

        # Build the 3x3 forward kinematics matrix: wheel_speeds -> body_velocity
        r = wheel_radius
        L = base_radius
        self.F = r * np.array([
            [np.sqrt(3) / 2, -np.sqrt(3) / 2, 0],
            [-1 / 2, -1 / 2, 1],
            [-1 / (3 * L), -1 / (3 * L), -1 / (3 * L)],
        ])
        self.F_inv = np.linalg.inv(self.F)

    def forward_kinematics(self, wheel_speeds: np.ndarray) -> np.ndarray:
        """Wheel angular velocities [left, right, back] (rad/s) -> body velocity [vx, vy, omega]."""
        return self.F @ wheel_speeds

    def inverse_kinematics(self, body_velocity: np.ndarray) -> np.ndarray:
        """Body velocity [vx, vy, omega] -> wheel angular velocities [left, right, back] (rad/s)."""
        return self.F_inv @ body_velocity


# ============ LeKiwi Bridge ============


@dataclass
class LeKiwiState:
    """Combined LeKiwi base + SO-101 arm state."""
    timestamp: float
    # Arm state
    arm_joint_names: List[str]
    arm_joint_positions: List[float]
    gripper_position: float  # 0.0=open, 1.0=closed
    # Base state
    wheel_positions: Dict[str, float]  # radians


class LeKiwiROS2Bridge(Node):
    """
    LeKiwi ROS2 Communication Bridge.

    Extends SO-101 arm control with omnidirectional base control.
    Subscribes to /joint_states for both arm and wheel feedback.
    Publishes to /cmd_vel for base and /joint_commands for arm.
    """

    WHEEL_JOINT_NAMES = ["left_wheel_joint", "rear_wheel_joint", "right_wheel_joint"]

    # Base velocity limits
    MAX_LINEAR_SPEED = 0.3    # m/s
    MAX_ANGULAR_SPEED = 1.57  # rad/s (~90 deg/s)

    # Default MuJoCo model: combined LeKiwi + SO-ARM100 scene
    DEFAULT_MODEL_PATH = "lekiwi_scene.xml"

    def __init__(self, node_name: str = "lekiwi_mcp_bridge"):
        super().__init__(node_name)

        self._arm_bridge = SO101ROS2Bridge.__new__(SO101ROS2Bridge)
        # Initialize the arm bridge's attributes without calling Node.__init__ again
        self._arm_bridge._current_state = None
        self._arm_bridge._connected = False
        self._arm_bridge._logger = self.get_logger()

        self._kinematics = OmniBaseKinematics()
        self._wheel_positions: Dict[str, float] = {}
        self._current_state: Optional[LeKiwiState] = None
        self._connected = False

        # ROS2 Subscribers
        self._joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        # ROS2 Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._joint_cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)

        # Wire the arm bridge's publisher to ours
        self._arm_bridge._joint_cmd_pub = self._joint_cmd_pub

        self.get_logger().info("LeKiwi MCP Bridge initialized (base + SO-101 arm)")

    def _joint_state_callback(self, msg: JointState):
        """Process joint states for both arm and base wheels."""
        arm_names = []
        arm_positions = []
        wheel_positions = {}

        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            if name in SO101ROS2Bridge.ALL_JOINT_NAMES:
                arm_names.append(name)
                arm_positions.append(pos)
            elif name in self.WHEEL_JOINT_NAMES:
                wheel_positions[name] = pos

        # Compute gripper normalized value
        gripper_pos = 0.0
        if SO101ROS2Bridge.GRIPPER_JOINT_NAME in arm_names:
            idx = arm_names.index(SO101ROS2Bridge.GRIPPER_JOINT_NAME)
            raw = arm_positions[idx]
            gripper_range = SO101ROS2Bridge.GRIPPER_CLOSED - SO101ROS2Bridge.GRIPPER_OPEN
            gripper_pos = max(0.0, min(1.0, (raw - SO101ROS2Bridge.GRIPPER_OPEN) / gripper_range))

        timestamp = self.get_clock().now().seconds_nanoseconds()[0]

        self._current_state = LeKiwiState(
            timestamp=timestamp,
            arm_joint_names=arm_names,
            arm_joint_positions=arm_positions,
            gripper_position=gripper_pos,
            wheel_positions=wheel_positions,
        )

        # Also update the arm bridge's state for reuse of SO-101 tools
        self._arm_bridge._current_state = SO101State(
            timestamp=timestamp,
            joint_names=arm_names,
            joint_positions=arm_positions,
            joint_velocities=[],
            joint_efforts=[],
            gripper_position=gripper_pos,
        )

        self._wheel_positions = wheel_positions

    def connect(self) -> bool:
        self._connected = True
        self._arm_bridge._connected = True
        return True

    def disconnect(self):
        self._connected = False
        self._arm_bridge._connected = False

    def publish_base_velocity(self, vx: float, vy: float, omega: float) -> bool:
        """Publish base velocity command as Twist on /cmd_vel."""
        try:
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = omega
            self._cmd_vel_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish base velocity: {e}")
            return False

    # SO-101 gripper servo is mounted inverted vs MuJoCo joint convention.
    # Negate gripper value only when publishing to the motor bridge.
    GRIPPER_INVERTED = True

    def publish_arm_command(self, positions: Dict[str, float]) -> bool:
        """Publish arm joint position command via /joint_commands."""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            pub_positions = dict(positions)
            if self.GRIPPER_INVERTED and SO101ROS2Bridge.GRIPPER_JOINT_NAME in pub_positions:
                pub_positions[SO101ROS2Bridge.GRIPPER_JOINT_NAME] = (
                    -pub_positions[SO101ROS2Bridge.GRIPPER_JOINT_NAME]
                )
            msg.name = list(pub_positions.keys())
            msg.position = list(pub_positions.values())
            self._joint_cmd_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish arm command: {e}")
            return False

    def validate_base_velocity(
        self, vx: float, vy: float, omega: float
    ) -> Tuple[bool, str]:
        """Validate base velocity against safety limits."""
        speed = (vx**2 + vy**2) ** 0.5
        if speed > self.MAX_LINEAR_SPEED:
            return False, (
                f"Linear speed {speed:.3f} m/s exceeds limit {self.MAX_LINEAR_SPEED} m/s"
            )
        if abs(omega) > self.MAX_ANGULAR_SPEED:
            return False, (
                f"Angular speed {abs(omega):.3f} rad/s exceeds limit "
                f"{self.MAX_ANGULAR_SPEED} rad/s"
            )
        return True, "OK"

    def validate_arm_positions(self, positions: Dict[str, float]) -> Tuple[bool, str]:
        """Delegate to SO-101 arm validation."""
        return self._arm_bridge.validate_joint_positions(positions)

    def get_current_state(self) -> Optional[LeKiwiState]:
        return self._current_state

    def get_arm_state(self) -> Optional[SO101State]:
        return self._arm_bridge.get_current_state()

    def init_firewall(self, model_path: Optional[str] = None) -> bool:
        """
        Initialize the Digital Twin Firewall with the combined LeKiwi+arm MuJoCo model.

        Delegates arm trajectory validation to the SO-101 bridge's firewall method,
        using the combined scene model that includes both base and arm.
        """
        if model_path is None:
            # Find specs directory relative to this package
            specs_dir = Path(__file__).resolve().parent.parent / "specs"
            if (specs_dir / self.DEFAULT_MODEL_PATH).exists():
                model_path = str(specs_dir / self.DEFAULT_MODEL_PATH)
            else:
                # Fallback: try rosclaw core package
                try:
                    import rosclaw
                    specs_dir = Path(rosclaw.__file__).parent / "specs"
                    model_path = str(specs_dir / self.DEFAULT_MODEL_PATH)
                except ImportError:
                    logger.warning("Model specs not found — firewall disabled")
                    return False

        # Initialize firewall on the arm bridge (reuses its validate_with_firewall)
        return self._arm_bridge.init_firewall(model_path)

    def validate_arm_with_firewall(
        self, positions: Dict[str, float]
    ) -> Tuple[bool, str]:
        """Validate arm positions through the MuJoCo Digital Twin."""
        return self._arm_bridge.validate_with_firewall(positions)


# Global bridge
_bridge: Optional[LeKiwiROS2Bridge] = None
_spin_thread: Optional[threading.Thread] = None


def init_ros():
    if not rclpy.ok():
        rclpy.init()


def shutdown_ros():
    if rclpy.ok():
        rclpy.shutdown()


# ============ MCP Tools — Connection ============


@mcp.tool()
async def connect_lekiwi() -> str:
    """
    Connect to LeKiwi robot (base + SO-101 arm) via ROS2.

    Initializes ROS2 node and starts state monitoring.
    Requires lekiwi_motor_bridge running with use_so101:=true.
    """
    global _bridge, _spin_thread

    if _bridge is not None:
        return "Already connected to LeKiwi"

    try:
        init_ros()
        _bridge = LeKiwiROS2Bridge()
        _bridge.connect()

        def spin_node():
            while rclpy.ok() and _bridge:
                rclpy.spin_once(_bridge, timeout_sec=0.1)

        _spin_thread = threading.Thread(target=spin_node, daemon=True)
        _spin_thread.start()

        # Initialize Digital Twin Firewall (non-fatal if unavailable)
        if _bridge.init_firewall():
            return "Connected to LeKiwi (base + SO-101 arm) via ROS2 (Digital Twin Firewall active)"
        return "Connected to LeKiwi (base + SO-101 arm) via ROS2 (no firewall)"

    except Exception as e:
        return f"Failed to connect: {e}"


@mcp.tool()
async def disconnect_lekiwi() -> str:
    """Disconnect from LeKiwi robot."""
    global _bridge, _spin_thread

    if _bridge:
        _bridge.disconnect()
        _bridge.destroy_node()
        _bridge = None
        _spin_thread = None
        return "Disconnected from LeKiwi"

    return "Not connected"


# ============ MCP Tools — Base Control ============


@mcp.tool()
async def move_base(
    vx: float = 0.0,
    vy: float = 0.0,
    omega: float = 0.0,
    duration: float = 1.0,
) -> str:
    """
    Move the LeKiwi base with given velocities for a duration, then stop.

    Args:
        vx: Forward velocity in m/s (positive = forward)
        vy: Lateral velocity in m/s (positive = left)
        omega: Rotational velocity in rad/s (positive = counter-clockwise)
        duration: How long to move in seconds (max 10.0)
    """
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    if not (0.0 < duration <= 10.0):
        return "Error: Duration must be between 0.0 and 10.0 seconds"

    valid, msg = _bridge.validate_base_velocity(vx, vy, omega)
    if not valid:
        return f"Safety check failed: {msg}"

    _bridge.publish_base_velocity(vx, vy, omega)
    await asyncio.sleep(duration)
    _bridge.publish_base_velocity(0.0, 0.0, 0.0)

    return f"Base moved (vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}) for {duration:.1f}s"


@mcp.tool()
async def stop_base() -> str:
    """Stop the LeKiwi base immediately (zero velocity)."""
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    _bridge.publish_base_velocity(0.0, 0.0, 0.0)
    return "Base stopped"


# ============ MCP Tools — Arm Control ============


@mcp.tool()
async def move_arm_to_home() -> str:
    """Move SO-101 arm to home position (all joints at zero)."""
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    valid, msg = _bridge.validate_arm_positions(SO101ROS2Bridge.HOME_POSITION)
    if not valid:
        return f"Safety check failed: {msg}"

    safe, fw_msg = _bridge.validate_arm_with_firewall(SO101ROS2Bridge.HOME_POSITION)
    if not safe:
        return fw_msg

    success = _bridge.publish_arm_command(SO101ROS2Bridge.HOME_POSITION)
    if success:
        return "Moving arm to home position"
    return "Failed to send command"


@mcp.tool()
async def move_arm_joint(joint_name: str, target_position: float, speed: float = 1.0) -> str:
    """
    Move a single arm joint to target position.

    Args:
        joint_name: One of: shoulder_pan, shoulder_lift, elbow_flex,
                    wrist_flex, wrist_roll, gripper
        target_position: Target angle in radians
        speed: Movement speed scale (0.1 to 1.0)
    """
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    if joint_name not in SO101ROS2Bridge.ALL_JOINT_NAMES:
        return f"Error: Unknown joint '{joint_name}'. Valid: {SO101ROS2Bridge.ALL_JOINT_NAMES}"

    valid, msg = _bridge.validate_arm_positions({joint_name: target_position})
    if not valid:
        return f"Safety check failed: {msg}"

    if not (0.1 <= speed <= 1.0):
        return "Error: Speed must be between 0.1 and 1.0"

    # Build full command from current state + target override
    arm_state = _bridge.get_arm_state()
    if arm_state:
        positions = dict(zip(arm_state.joint_names, arm_state.joint_positions))
    else:
        positions = dict(SO101ROS2Bridge.HOME_POSITION)

    positions[joint_name] = target_position

    safe, fw_msg = _bridge.validate_arm_with_firewall(positions)
    if not safe:
        return fw_msg

    success = _bridge.publish_arm_command(positions)
    if success:
        await asyncio.sleep(2.0 / speed)
        return f"Moved {joint_name} to {target_position:.3f} rad"
    return "Failed to send command"


@mcp.tool()
async def move_arm_joints(positions: Dict[str, float], speed: float = 1.0) -> str:
    """
    Move multiple arm joints simultaneously.

    Args:
        positions: Dictionary of {joint_name: target_position_rad}
        speed: Movement speed scale (0.1 to 1.0)
    """
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    valid, msg = _bridge.validate_arm_positions(positions)
    if not valid:
        return f"Safety check failed: {msg}"

    if not (0.1 <= speed <= 1.0):
        return "Error: Speed must be between 0.1 and 1.0"

    safe, fw_msg = _bridge.validate_arm_with_firewall(positions)
    if not safe:
        return fw_msg

    success = _bridge.publish_arm_command(positions)
    if success:
        await asyncio.sleep(3.0 / speed)
        return f"Moved {len(positions)} joints"
    return "Failed to send command"


@mcp.tool()
async def open_gripper() -> str:
    """Open the SO-101 gripper fully."""
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    success = _bridge.publish_arm_command({"gripper": SO101ROS2Bridge.GRIPPER_OPEN})
    if success:
        await asyncio.sleep(1.0)
        return "Gripper opened"
    return "Failed to send command"


@mcp.tool()
async def close_gripper() -> str:
    """Close the SO-101 gripper fully."""
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    success = _bridge.publish_arm_command({"gripper": SO101ROS2Bridge.GRIPPER_CLOSED})
    if success:
        await asyncio.sleep(1.0)
        return "Gripper closed"
    return "Failed to send command"


@mcp.tool()
async def stop_arm() -> str:
    """Stop arm motion by commanding current positions (hold in place)."""
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    arm_state = _bridge.get_arm_state()
    if arm_state is None:
        return "Error: No arm state data"

    positions = dict(zip(arm_state.joint_names, arm_state.joint_positions))
    success = _bridge.publish_arm_command(positions)
    if success:
        return "Arm holding current position"
    return "Failed to send stop command"


@mcp.tool()
async def stop_all() -> str:
    """Stop all motion — base and arm."""
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    _bridge.publish_base_velocity(0.0, 0.0, 0.0)

    arm_state = _bridge.get_arm_state()
    if arm_state:
        positions = dict(zip(arm_state.joint_names, arm_state.joint_positions))
        _bridge.publish_arm_command(positions)

    return "All motion stopped"


# ============ MCP Tools — State ============


@mcp.tool()
async def get_robot_state() -> str:
    """
    Get full LeKiwi state — arm joint positions, gripper, and wheel positions.
    """
    if _bridge is None:
        return "Error: Not connected to LeKiwi"

    state = _bridge.get_current_state()
    if state is None:
        return "No state data available (waiting for /joint_states)"

    lines = [f"LeKiwi State (t={state.timestamp}):", "", "  Arm Joints:"]
    for name, pos in zip(state.arm_joint_names, state.arm_joint_positions):
        limits = SO101ROS2Bridge.JOINT_LIMITS.get(name, (0, 0))
        lines.append(
            f"    {name}: {pos:.4f} rad  (limits: {limits[0]:.3f} to {limits[1]:.3f})"
        )
    lines.append(f"  Gripper: {state.gripper_position * 100:.0f}% closed")

    lines.append("")
    lines.append("  Base Wheels:")
    for name in LeKiwiROS2Bridge.WHEEL_JOINT_NAMES:
        pos = state.wheel_positions.get(name, 0.0)
        lines.append(f"    {name}: {pos:.4f} rad")

    return "\n".join(lines)


# ============ MCP Resources ============


@mcp.resource("lekiwi://status")
async def get_lekiwi_status() -> str:
    """Get LeKiwi combined arm + base status."""
    if _bridge is None:
        return "Not connected to LeKiwi"

    state = _bridge.get_current_state()
    if state is None:
        return "No state data available"

    arm_info = []
    for name, pos in zip(state.arm_joint_names, state.arm_joint_positions):
        arm_info.append(f"    {name}: {pos:.4f} rad")

    wheel_info = []
    for name in LeKiwiROS2Bridge.WHEEL_JOINT_NAMES:
        pos = state.wheel_positions.get(name, 0.0)
        wheel_info.append(f"    {name}: {pos:.4f} rad")

    return (
        f"LeKiwi Status:\n"
        f"  Timestamp: {state.timestamp}\n\n"
        f"  Arm Joints:\n" + "\n".join(arm_info) + "\n"
        f"  Gripper: {state.gripper_position * 100:.1f}% closed\n\n"
        f"  Base Wheels:\n" + "\n".join(wheel_info)
    )


@mcp.resource("lekiwi://joints")
async def get_lekiwi_joint_limits() -> str:
    """Get all joint limits — arm and base."""
    info = ["LeKiwi Joint Limits:", "=" * 50, "", "  Arm (SO-101):"]

    for joint in SO101ROS2Bridge.ALL_JOINT_NAMES:
        min_val, max_val = SO101ROS2Bridge.JOINT_LIMITS[joint]
        info.append(f"    {joint}: [{min_val:.3f}, {max_val:.3f}] rad")

    info.append("")
    info.append("  Base:")
    info.append(f"    Max linear speed: {LeKiwiROS2Bridge.MAX_LINEAR_SPEED} m/s")
    info.append(f"    Max angular speed: {LeKiwiROS2Bridge.MAX_ANGULAR_SPEED} rad/s")
    info.append(f"    Wheel radius: {OmniBaseKinematics().wheel_radius} m")
    info.append(f"    Base radius: {OmniBaseKinematics().base_radius} m")

    return "\n".join(info)


@mcp.resource("lekiwi://connection")
async def get_lekiwi_connection_status() -> str:
    """Get ROS2 connection status."""
    if _bridge and _bridge._connected:
        return "Connected to LeKiwi (base + SO-101 arm) via ROS2"
    return "Disconnected"


if __name__ == "__main__":
    mcp.run(transport="stdio")
