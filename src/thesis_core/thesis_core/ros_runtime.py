"""Shared lifecycle helpers for ROS 2 Python nodes."""

import rclpy
from rclpy.executors import ExternalShutdownException


_SHUTDOWN_TAKE_MESSAGE_ERROR = (
    'Unable to convert call argument to Python object '
    '(compile in debug mode for details)'
)


def _is_shutdown_take_message_error(exception):
    """Recognize the rclpy Humble subscription race seen during SIGINT."""
    return (
        isinstance(exception, RuntimeError)
        and str(exception) == _SHUTDOWN_TAKE_MESSAGE_ERROR
    )


def spin_node(node):
    """Spin and release a node, treating requested shutdown as successful."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exception:
        # Timers can race with SIGINT after the ROS context is invalidated.
        # Preserve genuine runtime failures while suppressing only that
        # expected shutdown race.  Humble can leave the context reporting OK
        # while take_message raises its fixed conversion error during SIGINT.
        if rclpy.ok() and not _is_shutdown_take_message_error(exception):
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception:
            if rclpy.ok():
                raise
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
