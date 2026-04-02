"""Tests for LeKiwi MCP server — validation logic (no ROS2 required).

ROS2 and MCP modules are mocked via conftest.py.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "rosclaw-so101-ros2-mcp" / "src"))

from lekiwi_mcp_server import LeKiwiROS2Bridge


class TestBaseVelocityValidation:
    """Test base velocity validation against safety limits."""

    def _make_bridge(self):
        return object.__new__(LeKiwiROS2Bridge)

    def test_zero_velocity_valid(self):
        bridge = self._make_bridge()
        valid, msg = bridge.validate_base_velocity(0.0, 0.0, 0.0)
        assert valid

    def test_forward_within_limit(self):
        bridge = self._make_bridge()
        valid, _ = bridge.validate_base_velocity(0.2, 0.0, 0.0)
        assert valid

    def test_forward_exceeds_limit(self):
        bridge = self._make_bridge()
        valid, msg = bridge.validate_base_velocity(0.5, 0.0, 0.0)
        assert not valid
        assert "Linear speed" in msg

    def test_diagonal_within_limit(self):
        bridge = self._make_bridge()
        # sqrt(0.2^2 + 0.2^2) = 0.283 < 0.3
        valid, _ = bridge.validate_base_velocity(0.2, 0.2, 0.0)
        assert valid

    def test_diagonal_exceeds_limit(self):
        bridge = self._make_bridge()
        # sqrt(0.25^2 + 0.25^2) = 0.354 > 0.3
        valid, msg = bridge.validate_base_velocity(0.25, 0.25, 0.0)
        assert not valid

    def test_rotation_within_limit(self):
        bridge = self._make_bridge()
        valid, _ = bridge.validate_base_velocity(0.0, 0.0, 1.0)
        assert valid

    def test_rotation_exceeds_limit(self):
        bridge = self._make_bridge()
        valid, msg = bridge.validate_base_velocity(0.0, 0.0, 2.0)
        assert not valid
        assert "Angular speed" in msg
