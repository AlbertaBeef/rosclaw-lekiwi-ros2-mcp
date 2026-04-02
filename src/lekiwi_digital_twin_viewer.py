"""
ROSClaw LeKiwi Digital Twin Viewer

Live MuJoCo visualization that mirrors the real robot's joint states.
Subscribes to /joint_states and updates the MuJoCo model in real-time.

Uses the passive viewer mode — no physics simulation, just visualization.
Joint positions are set directly from ROS2 feedback.

Usage:
    source ~/rosclaw-venv/bin/activate
    source /opt/ros/jazzy/setup.bash
    python3 /media/abbeefai/TheExpanse/rosclaw/rosclaw-lekiwi-ros2-mcp/src/lekiwi_digital_twin_viewer.py
"""

import threading
from pathlib import Path

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ROS2 joint name -> MuJoCo joint name mapping
ROS2_TO_MUJOCO_JOINT = {
    "shoulder_pan": "Rotation",
    "shoulder_lift": "Pitch",
    "elbow_flex": "Elbow",
    "wrist_flex": "Wrist_Pitch",
    "wrist_roll": "Wrist_Roll",
    "gripper": "Jaw",
}

# Actual servo-space gripper range (negated, as seen after inversion)
# Derived from calibration: homing_offset=2788, range_min=2053
# Closed: -(2053-2788)/651.9 = +1.128, Open: -(2902-2788)/651.9 = -0.175
NEGATED_SERVO_CLOSED = 1.128
NEGATED_SERVO_OPEN = -0.175

# MuJoCo Jaw joint range (SO-100 mesh geometry)
MUJOCO_JAW_CLOSED = 0.0
MUJOCO_JAW_OPEN = 0.6

ROS2_TO_MUJOCO_WHEEL = {
    "left_wheel_joint": "base_left_wheel_joint",
    "rear_wheel_joint": "base_back_wheel_joint",
    "right_wheel_joint": "base_right_wheel_joint",
}


class DigitalTwinViewer(Node):
    """ROS2 node that feeds joint states into a live MuJoCo passive viewer."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        super().__init__("digital_twin_viewer")
        self.model = model
        self.data = data
        self._lock = threading.Lock()

        # Build joint index lookup: mujoco_name -> qpos index
        self._joint_qpos_idx = {}
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self._joint_qpos_idx[name] = model.jnt_qposadr[joint_id]

        self.get_logger().info(
            f"Digital Twin Viewer initialized — tracking {len(self._joint_qpos_idx)} joints"
        )

        # Subscribe to joint states
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState):
        """Update MuJoCo qpos with real robot joint positions."""
        with self._lock:
            for i, ros_name in enumerate(msg.name):
                if i >= len(msg.position):
                    break
                pos = msg.position[i]

                # Map arm joints
                if ros_name in ROS2_TO_MUJOCO_JOINT:
                    mj_name = ROS2_TO_MUJOCO_JOINT[ros_name]
                    if mj_name in self._joint_qpos_idx:
                        if ros_name == "gripper":
                            # Map inverted servo-space to MuJoCo Jaw range
                            # Motor bridge reports un-negated servo radians; negate here
                            negated = -pos
                            servo_range = NEGATED_SERVO_CLOSED - NEGATED_SERVO_OPEN
                            t = (negated - NEGATED_SERVO_OPEN) / servo_range  # 0=open, 1=closed
                            t = max(0.0, min(1.0, t))
                            pos = MUJOCO_JAW_OPEN + t * (MUJOCO_JAW_CLOSED - MUJOCO_JAW_OPEN)
                        self.data.qpos[self._joint_qpos_idx[mj_name]] = pos

                # Map wheel joints (negate: servo direction is inverted vs MuJoCo)
                elif ros_name in ROS2_TO_MUJOCO_WHEEL:
                    mj_name = ROS2_TO_MUJOCO_WHEEL[ros_name]
                    if mj_name in self._joint_qpos_idx:
                        self.data.qpos[self._joint_qpos_idx[mj_name]] = -pos


def main():
    # Determine model path — look in this package's specs/ first
    specs_dir = Path(__file__).resolve().parent.parent / "specs"
    model_path = str(specs_dir / "lekiwi_scene.xml")
    if not Path(model_path).exists():
        # Fallback: try current working directory
        model_path = "lekiwi_scene.xml"
    if not Path(model_path).exists():
        # Fallback: try rosclaw core package
        try:
            import rosclaw
            model_path = str(Path(rosclaw.__file__).parent / "specs" / "lekiwi_scene.xml")
        except ImportError:
            pass

    print(f"Loading MuJoCo model: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Run mj_forward once to initialize derived quantities
    mujoco.mj_forward(model, data)

    # Initialize ROS2
    rclpy.init()
    viewer_node = DigitalTwinViewer(model, data)

    # Spin ROS2 in background thread
    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(viewer_node, timeout_sec=0.02)

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    print("Digital Twin Viewer running — mirrors real robot joint states")
    print("Close the viewer window to exit")

    # Launch passive viewer — no physics stepping, just renders current qpos
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            with viewer_node._lock:
                # Update kinematics from current qpos (no contact/constraint solving)
                mujoco.mj_kinematics(model, data)
            viewer.sync()

    # Cleanup
    viewer_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
