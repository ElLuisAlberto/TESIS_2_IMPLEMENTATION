"""Shared lifecycle helpers for ROS 2 Python nodes."""

import rclpy
from rclpy.executors import ExternalShutdownException


def spin_node(node):
    """Spin and release a node, treating requested shutdown as successful."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Timers can race with SIGINT after the ROS context is invalidated.
        # Preserve genuine runtime failures while suppressing only that
        # expected shutdown race.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            if rclpy.ok():
                raise
        if rclpy.ok():
            rclpy.shutdown()
