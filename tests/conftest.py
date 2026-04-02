"""Mock ROS2 and MCP modules for testing without installations."""

import sys
from types import ModuleType
from unittest.mock import MagicMock


def _setup_mocks():
    """Install mocks for rclpy, geometry_msgs, sensor_msgs, std_msgs, and mcp."""

    # Create a real Node base class so our bridge classes are real types
    class FakeNode:
        def __init__(self, name=""):
            pass

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_publisher(self, *args, **kwargs):
            return MagicMock()

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            clock = MagicMock()
            clock.now().seconds_nanoseconds.return_value = (0, 0)
            clock.now().to_msg.return_value = MagicMock()
            return clock

        def destroy_node(self):
            pass

    # Create a real FastMCP so decorators work
    class FakeFastMCP:
        def __init__(self, name=""):
            self.name = name

        def tool(self):
            def decorator(fn):
                return fn
            return decorator

        def resource(self, uri):
            def decorator(fn):
                return fn
            return decorator

        def run(self, **kwargs):
            pass

    # rclpy
    rclpy_mod = ModuleType("rclpy")
    rclpy_mod.ok = lambda: True
    rclpy_mod.init = lambda **kw: None
    rclpy_mod.shutdown = lambda: None
    rclpy_mod.spin_once = lambda *a, **kw: None
    sys.modules["rclpy"] = rclpy_mod

    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    sys.modules["rclpy.node"] = rclpy_node

    rclpy_action = ModuleType("rclpy.action")
    sys.modules["rclpy.action"] = rclpy_action

    # geometry_msgs
    geo_mod = ModuleType("geometry_msgs")
    geo_msg = ModuleType("geometry_msgs.msg")
    geo_msg.Twist = MagicMock
    geo_msg.Pose = MagicMock
    geo_msg.PoseStamped = MagicMock
    geo_mod.msg = geo_msg
    sys.modules["geometry_msgs"] = geo_mod
    sys.modules["geometry_msgs.msg"] = geo_msg

    # sensor_msgs
    sen_mod = ModuleType("sensor_msgs")
    sen_msg = ModuleType("sensor_msgs.msg")
    sen_msg.JointState = MagicMock
    sen_mod.msg = sen_msg
    sys.modules["sensor_msgs"] = sen_mod
    sys.modules["sensor_msgs.msg"] = sen_msg

    # std_msgs
    std_mod = ModuleType("std_msgs")
    std_msg = ModuleType("std_msgs.msg")
    std_msg.Float64MultiArray = MagicMock
    std_mod.msg = std_msg
    sys.modules["std_msgs"] = std_mod
    sys.modules["std_msgs.msg"] = std_msg

    # mcp
    mcp_mod = ModuleType("mcp")
    mcp_server = ModuleType("mcp.server")
    mcp_fastmcp = ModuleType("mcp.server.fastmcp")
    mcp_fastmcp.FastMCP = FakeFastMCP
    mcp_server.fastmcp = mcp_fastmcp
    mcp_mod.server = mcp_server
    sys.modules["mcp"] = mcp_mod
    sys.modules["mcp.server"] = mcp_server
    sys.modules["mcp.server.fastmcp"] = mcp_fastmcp


# Run at import time — before any test module imports the server code
_setup_mocks()
