"""Front-axle Stanley law. CARLA yaw and positive steering point towards +y."""
import math


def steering(heading, path_heading, signed_error, speed, max_angle, gain):
    """Return normalized steering before the shared actuator smoothing/limit.

    signed_error is positive on the path's right side in CARLA coordinates.
    """
    heading_error = math.atan2(math.sin(path_heading-heading),
                               math.cos(path_heading-heading))
    angle = heading_error - math.atan2(gain*signed_error, max(speed, 0.1))
    return angle/max_angle
