"""Logging helpers for physical-stack readiness transitions."""


def log_readiness_transition(logger, ready, text):
    """
    Log READY and NOT_READY from distinct ROS 2 call sites.

    ROS 2 Humble associates a logger call site with its first severity.  Keep
    the INFO and WARN invocations on separate source lines so a transition
    between the two states cannot terminate the readiness node.
    """
    if ready:
        logger.info(text)
    else:
        logger.warning(text)
