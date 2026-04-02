"""Tests for OmniBaseKinematics — no ROS2 required.

ROS2 and MCP modules are mocked via conftest.py.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "rosclaw-so101-ros2-mcp" / "src"))

import numpy as np

from lekiwi_mcp_server import OmniBaseKinematics


class TestOmniBaseKinematics:
    """Test omnidirectional base kinematics."""

    def setup_method(self):
        self.kin = OmniBaseKinematics(wheel_radius=0.054, base_radius=0.125)

    def test_zero_wheels_zero_velocity(self):
        result = self.kin.forward_kinematics(np.array([0.0, 0.0, 0.0]))
        np.testing.assert_allclose(result, [0.0, 0.0, 0.0], atol=1e-10)

    def test_inverse_then_forward_roundtrip(self):
        body_vel = np.array([0.1, 0.0, 0.0])
        wheels = self.kin.inverse_kinematics(body_vel)
        recovered = self.kin.forward_kinematics(wheels)
        np.testing.assert_allclose(recovered, body_vel, atol=1e-10)

    def test_lateral_roundtrip(self):
        body_vel = np.array([0.0, 0.1, 0.0])
        wheels = self.kin.inverse_kinematics(body_vel)
        recovered = self.kin.forward_kinematics(wheels)
        np.testing.assert_allclose(recovered, body_vel, atol=1e-10)

    def test_rotation_roundtrip(self):
        body_vel = np.array([0.0, 0.0, 0.5])
        wheels = self.kin.inverse_kinematics(body_vel)
        recovered = self.kin.forward_kinematics(wheels)
        np.testing.assert_allclose(recovered, body_vel, atol=1e-10)

    def test_combined_motion_roundtrip(self):
        body_vel = np.array([0.15, -0.05, 0.3])
        wheels = self.kin.inverse_kinematics(body_vel)
        recovered = self.kin.forward_kinematics(wheels)
        np.testing.assert_allclose(recovered, body_vel, atol=1e-10)

    def test_pure_rotation_equal_wheel_speeds(self):
        """Pure rotation should produce equal-magnitude wheel speeds."""
        body_vel = np.array([0.0, 0.0, 1.0])
        wheels = self.kin.inverse_kinematics(body_vel)
        np.testing.assert_allclose(
            np.abs(wheels[0]), np.abs(wheels[1]), atol=1e-10
        )
        np.testing.assert_allclose(
            np.abs(wheels[1]), np.abs(wheels[2]), atol=1e-10
        )

    def test_matrices_are_inverse(self):
        """F and F_inv should be true inverses."""
        identity = self.kin.F @ self.kin.F_inv
        np.testing.assert_allclose(identity, np.eye(3), atol=1e-10)
