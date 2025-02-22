import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import BPoly


class BezierCurve:
    def __init__(self):
        self._xy_control_points = np.array(
            [-1, -1.4, -1.5, -1.5, -1.5, 0, 0, 0, 1.5, 1.5, 1.4, 1], dtype=float
        ).reshape((-1, 1))

        self._z_control_points = np.array(
            [0, 0, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 1.1, 0, 0], dtype=float
        ).reshape((-1, 1))

        self._t_range = np.array([0, 1], dtype=float)

    def evaluate(self, t: float, xy_length: float, y_angle: float, z_height: float):
        if xy_length == 0:
            return np.zeros(3)
        xy_control_points = self._xy_control_points * xy_length
        x_poly = BPoly.construct_fast(
            xy_control_points * np.cos(y_angle), self._t_range
        )
        y_poly = BPoly.construct_fast(
            xy_control_points * np.sin(y_angle), self._t_range
        )
        z_poly = BPoly.construct_fast(self._z_control_points * z_height, self._t_range)
        return np.array([x_poly(t), y_poly(t), z_poly(t)])


class SineCurve:
    def evaluate(self, t: float, xy_length: float, y_angle: float, z_height: float):
        # if len(t) == 0:
        #    return np.zeros((0, 3))
        xy = xy_length * (1.0 - 2.0 * t)
        x, y = xy * np.cos(y_angle), xy * np.sin(y_angle)
        # z = np.zeros(len(t))
        z = 0
        if xy_length != 0:
            z = -z_height * np.cos((np.pi * (x + y)) / (2.0 * xy_length))
        return np.array([x, y, z])


def _step_length(velocity: float, stance_time: float) -> float:
    return (velocity * stance_time) / 2


class Gait:
    def __init__(
        self,
        foot_rest_poses: NDArray,
        target_speed: float = 0.1,
        lateral_rotation_angle: float = 0,
        yaw_rate: float = 0,
        swing_time: float = 0.2,
        phase_lag: tuple[float, float, float, float] = (0, 0.5, 0.5, 0),
        clearance_height: float = 0.03,
        penetration_depth: float = 0.005,
    ):
        self.foot_rest_pose = foot_rest_poses
        self.lateral_rotation_angle = lateral_rotation_angle
        self.yaw_rate = yaw_rate
        self.target_speed = target_speed
        self.swing_time = swing_time
        self.leg_lag = np.array(phase_lag, dtype=float)
        self.clearance_height = clearance_height
        self.penetration_depth = penetration_depth

        self._time = 0.0
        self._swing_curve = BezierCurve()
        self._stance_curve = SineCurve()
        self._rest_xy_angle = np.arctan2(
            self.foot_rest_pose[:, 1], self.foot_rest_pose[:, 0]
        )
        self._prev_foot_pose = self.foot_rest_pose.copy()

    #def stance_time(self) -> float:
    #    return min(self.swing_time * 1.3, 2 * abs(self.step_length / self.target_speed))

    def stance_time(self) -> float:
        velocity = max(abs(self.target_speed), abs(self.yaw_rate))
        return self.swing_time * (1 - velocity / 1.5)

    def stride_time(self) -> float:
        return self.swing_time + self.stance_time()

    def leg_phase(self) -> NDArray:
        swing_time = self.swing_time
        stance_time = self.stance_time()
        stride_time = self.stride_time()

        leg_time = (self._time + ((1 - self.leg_lag) * stride_time)) % stride_time
        leg_phase = np.array(
            [
                (
                    x / stance_time
                    if x < stance_time
                    else (x - stance_time) / swing_time + 1
                )
                for x in leg_time
            ]
        )
        return leg_phase

    def compute_foot_positions(self, dt: float):
        stride_time = self.stride_time()
        self._time = (self._time + dt) % stride_time

        if abs(self.target_speed) < 0.005 and abs(self.yaw_rate) < 0.005:
            return self.foot_rest_pose.copy()

        leg_phase = self.leg_phase()
        is_stance = leg_phase < 1
        leg_phase[~is_stance] -= 1

        stance_time = self.stance_time()
        fwd_step_length = _step_length(self.target_speed, stance_time)
        yaw_step_length = _step_length(self.yaw_rate, stance_time)

        rel_prev_foot_pose = self._prev_foot_pose - self.foot_rest_pose
        rest_foot_mag = np.linalg.norm(self.foot_rest_pose[:, :2], axis=1)
        prev_foot_mag = np.linalg.norm(rel_prev_foot_pose[:, :2], axis=1)

        rel_xy_angle = np.arctan2(prev_foot_mag, rest_foot_mag)
        xy_angle = np.pi / 2 + rel_xy_angle + self._rest_xy_angle

        foot_poses = self.foot_rest_pose.copy()
        # print(self.swing_time, self.stance_time(), self.get_step_length(self.target_speed, self.stance_time()))

        for i in range(len(foot_poses)):
            curve = self._stance_curve if is_stance[i] else self._swing_curve
            height_scalar = (
                self.penetration_depth if is_stance[i] else self.clearance_height
            )

            foot_poses[i, :] += curve.evaluate(
                leg_phase[i],
                fwd_step_length,
                self.lateral_rotation_angle,
                height_scalar,
            )
            foot_poses[i, :] += curve.evaluate(
                leg_phase[i],
                yaw_step_length,
                xy_angle[i],
                height_scalar,
            )

        self._prev_foot_pose = foot_poses
        return foot_poses
