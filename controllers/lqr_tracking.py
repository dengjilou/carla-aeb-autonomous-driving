"""Discrete rear-axle kinematic LQR with curvature feedforward (SI units)."""
import math
import numpy as np


class LQR:
    def __init__(self, speed, wheelbase, dt=0.05, position_weight=0.2):
        if min(speed, wheelbase, dt, position_weight) <= 0:
            raise ValueError("Positive speed, geometry, timestep and weight required")
        self.wheelbase = wheelbase
        self.a = np.array([[1., speed*dt], [0., 1.]])
        self.b = np.array([[speed**2*dt**2/(2*wheelbase)], [speed*dt/wheelbase]])
        self.q = np.diag([position_weight, 1.])
        p = self.q.copy()
        for _ in range(10000):
            k = np.linalg.solve(np.eye(1)+self.b.T@p@self.b, self.b.T@p@self.a)
            new = self.q + self.a.T@p@(self.a-self.b@k)
            if np.max(abs(new-p)) < 1e-10:
                self.p = new
                self.k = np.linalg.solve(np.eye(1)+self.b.T@new@self.b, self.b.T@new@self.a)
                break
            p = new
        else:
            raise RuntimeError("Discrete Riccati iteration did not converge")

    def terms(self, lateral_error, heading_error, curvature):
        wrapped = math.atan2(math.sin(heading_error), math.cos(heading_error))
        feedback = float((self.k@np.array([lateral_error, wrapped]))[0])
        return math.atan(self.wheelbase*curvature), -feedback

    def steering(self, lateral_error, heading_error, curvature, max_angle):
        forward, feedback = self.terms(lateral_error, heading_error, curvature)
        return (forward+feedback)/max_angle


class ScheduledLQR:
    """Interpolate gains on a frozen 1..30 m/s grid; clamp outside that range."""
    def __init__(self, wheelbase, dt=0.05, position_weight=0.2):
        self.wheelbase = wheelbase
        self.speeds = np.arange(1., 31.)
        self.gains = np.array([LQR(v, wheelbase, dt, position_weight).k[0] for v in self.speeds])

    def gain(self, speed):
        if not math.isfinite(speed):
            raise ValueError("Finite speed required")
        return np.array([np.interp(speed, self.speeds, self.gains[:, i]) for i in range(2)])

    def steering(self, lateral_error, heading_error, curvature, max_angle, speed):
        wrapped = math.atan2(math.sin(heading_error), math.cos(heading_error))
        feedback = float(self.gain(speed) @ np.array([lateral_error, wrapped]))
        return (math.atan(self.wheelbase*curvature)-feedback)/max_angle
