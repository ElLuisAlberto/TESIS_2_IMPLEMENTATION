"""
Validation and rigid-frame transforms for one spherical obstacle estimate.

The message contract is source agnostic. A perception adapter may turn a depth
image or point cloud into a conservative sphere, while simulation can publish
the same estimate directly.
"""

import math


def validate_obstacle(
    center,
    velocity,
    radius,
    uncertainty,
    stamp_sec,
    now_sec,
    max_age_sec=0.5,
    future_tolerance_sec=0.05,
):
    """Raise ValueError unless an estimate is finite, bounded, and fresh."""
    point = tuple(float(value) for value in center)
    speed = tuple(float(value) for value in velocity)
    scalars = (float(radius), float(uncertainty), float(stamp_sec),
               float(now_sec), float(max_age_sec),
               float(future_tolerance_sec))
    if len(point) != 3 or len(speed) != 3:
        raise ValueError('center and velocity must contain three values')
    if not all(math.isfinite(value) for value in point + speed + scalars):
        raise ValueError('obstacle values must be finite')
    (radius, uncertainty, stamp_sec, now_sec, max_age_sec,
     future_tolerance_sec) = scalars
    if radius <= 0.0:
        raise ValueError('radius must be positive')
    if uncertainty < 0.0:
        raise ValueError('uncertainty must be non-negative')
    if max_age_sec <= 0.0 or future_tolerance_sec < 0.0:
        raise ValueError('age limits are invalid')
    age = now_sec - stamp_sec
    if age < -future_tolerance_sec:
        raise ValueError('obstacle timestamp is too far in the future')
    if age > max_age_sec:
        raise ValueError('obstacle estimate is stale')
    return point, speed, radius, uncertainty


def rotate_vector(vector, quaternion):
    """Rotate a 3-vector by a normalized ROS-order quaternion (x, y, z, w)."""
    vx, vy, vz = (float(value) for value in vector)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError('transform quaternion must be finite and non-zero')
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    # Optimized q*v*q^-1 form.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def transform_obstacle(center, velocity, translation, quaternion):
    """Transform a point and its free vector into the target frame."""
    rotated_center = rotate_vector(center, quaternion)
    rotated_velocity = rotate_vector(velocity, quaternion)
    offset = tuple(float(value) for value in translation)
    if len(offset) != 3 or not all(math.isfinite(v) for v in offset):
        raise ValueError('transform translation must be finite and 3D')
    return (
        tuple(a + b for a, b in zip(rotated_center, offset)),
        rotated_velocity,
    )
