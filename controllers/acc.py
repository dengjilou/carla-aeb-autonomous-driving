"""Simulator-independent constant-time-headway ACC and speed PID (SI units)."""


def clamp(value, low, high):
    return max(low, min(high, value))


class ACC:
    def __init__(self, gap_gain=0.2, relative_gain=0.8, standstill_gap=8.0, headway=1.8):
        self.gap_gain = gap_gain
        self.relative_gain = relative_gain
        self.standstill_gap = standstill_gap
        self.headway = headway

    def target_speed(self, speed, cruise, gap=None, closing=0.0):
        if gap is None:
            return cruise
        desired_gap = self.standstill_gap + self.headway * speed
        acceleration = clamp(self.gap_gain * (gap - desired_gap) - self.relative_gain * closing, -3.0, 2.0)
        # One-second speed reference horizon; the plant is sampled at 50 ms.
        return clamp(speed + acceleration, 0.0, cruise)


class SpeedPID:
    def __init__(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, target, speed, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")
        error = target - speed
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        candidate = clamp(self.integral + error * dt, -3.0, 3.0)
        command = 0.45 * error + 0.12 * candidate + 0.01 * derivative
        if -1.0 <= command <= 0.75:
            self.integral = candidate
        self.previous_error = error
        command = clamp(command, -1.0, 0.75)
        return max(0.0, command), max(0.0, -command)


def arbitrate(throttle, brake, aeb_brake, latched_brake):
    """Emergency intervention persists to the end of this experiment."""
    latched_brake = max(latched_brake, aeb_brake)
    if latched_brake > 0:
        return 0.0, max(brake, latched_brake), latched_brake
    return throttle, brake, latched_brake
