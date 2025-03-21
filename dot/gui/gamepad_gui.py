from dataclasses import dataclass
import dearpygui.dearpygui as dpg
import numpy as np
from scipy.spatial.transform import Rotation
import math

from dot.control.gait import Gait
from dot.control.gamepad import Gamepad
from dot.control.inverse_kinematics import RobotIK


class SmoothValue:
    def __init__(self, value=0):
        self.value = value
        self.target = value
        self.smooth_scalar = 0.85

    def update(self):
        s = self.smooth_scalar
        self.value = (1 - s) * self.target + s * self.value


class GamepadGui:
    def __init__(self, robot_ik: RobotIK, robot_gait: Gait):
        self._gamepad = Gamepad()
        self._robot_ik = robot_ik
        self._robot_gait = robot_gait
        self._use_gamepad = dpg.add_checkbox(label="Use Gamepad", default_value=False)
        self._init_smooth_values()

    def _init_smooth_values(self):
        self._translation = [SmoothValue() for _ in range(3)]
        self._euler_rad = [SmoothValue() for _ in range(3)]
        self._linear_velocity = SmoothValue()
        self._lateral_angle = SmoothValue()
        self._yaw_velocity = SmoothValue()

    def _update_smooth_values(self):
        values = [
            *self._translation,
            *self._euler_rad,
            self._linear_velocity,
            self._lateral_angle,
            self._yaw_velocity,
        ]
        for v in values:
            v.update()

    def _set_from_values(self):
        ik = self._robot_ik

        ik.translation += np.array([v.value for v in self._translation])

        euler_angles = ik.rotation.as_euler("XYZ", degrees=False)
        euler_angles += np.array([v.value for v in self._euler_rad])
        ik.rotation = Rotation.from_euler("XYZ", euler_angles, degrees=False)

        gait = self._robot_gait
        gait.target_speed = self._linear_velocity.value
        gait.lateral_rotation_angle = self._lateral_angle.value
        gait.yaw_rate = self._yaw_velocity.value

    def use_gamepad(self):
        return dpg.get_value(self._use_gamepad)

    def update(self):
        if not self.use_gamepad():
            return

        self._update_targets()
        self._update_smooth_values()
        self._set_from_values()

    def _update_targets(self):
        gamepad = self._gamepad
        left_axis = np.array([gamepad.left_hat_x, gamepad.left_hat_y])
        right_axis = np.array([gamepad.right_hat_x, gamepad.right_hat_y])
        for axis in [left_axis, right_axis]:
            for i in range(len(axis)):
                if abs(axis[i]) < 0.2:
                    axis[i] = 0
                else:
                    axis[i] += -0.2 if axis[i] > 0 else 0.2

        if gamepad.right_trigger > 0.5:
            # euler_angles = robot_ik.rotation.as_euler("XYZ", degrees=False)
            euler = self._euler_rad
            euler[0].target = np.interp(left_axis[0], [-0.8, 0.8], [-0.4, 0.4])
            euler[1].target = np.interp(left_axis[1], [-0.8, 0.8], [-0.5, 0.5])
            euler[2].target = np.interp(right_axis[0], [-0.8, 0.8], [-0.4, 0.4])

            # robot_ik.rotation = Rotation.from_euler("XYZ", euler_angles, degrees=False)
        elif gamepad.left_trigger > 0.5:
            trans = self._translation
            trans[0].target = np.interp(-left_axis[1], [-0.8, 0.8], [-0.03, 0.03])
            trans[1].target = np.interp(left_axis[0], [-0.8, 0.8], [-0.05, 0.05])
            trans[2].target = -np.interp(right_axis[1], [-0.8, 0.8], [-0.07, 0.07])
        else:
            velocity_frac = np.linalg.norm(left_axis)
            velocity_dir = 1 if left_axis[1] > 0 else -1
            lateral_dir = 1 if left_axis[0] > 0 else -1
            lateral_angle = abs(math.atan2(left_axis[0], abs(left_axis[1])))

            self._linear_velocity.target = velocity_dir * np.interp(
                velocity_frac, [0, 0.8], [0, 0.18]
            )

            self._lateral_angle.target = -velocity_dir * lateral_dir * lateral_angle
            self._yaw_velocity.target = -np.interp(
                right_axis[0], [-0.8, 0.8], [-0.06, 0.06]
            )
