from dataclasses import dataclass
import numpy as np

from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from multiprocessing import Process, Value
from scipy.spatial.transform import Rotation

import inputs
import math
import threading

class Gamepad:
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_HAT_VAL = math.pow(2, 15)

    def __init__(self):
        self.left_hat_y = 0
        self.left_hat_x = 0
        self.right_hat_y = 0
        self.right_hat_x = 0
        self.left_trigger = 0
        self.right_trigger = 0
        self.left_bumper = 0
        self.right_bumper = 0
        self.a = 0
        self.x = 0
        self.y = 0
        self.b = 0
        self.left_thumb = 0
        self.right_thumb = 0
        self.back = 0
        self.start = 0
        self.left_dpad = 0
        self.right_dpad = 0
        self.up_dpad = 0
        self.down_dpad = 0

        if len(inputs.devices.gamepads) > 0:
            self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
            self._monitor_thread.daemon = True
            self._monitor_thread.start()
        else:
            print("No gamepad detected.")


    def _monitor_controller(self):
        while True:
            events = inputs.get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.left_hat_y = event.state / Gamepad.MAX_HAT_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.left_hat_x = event.state / Gamepad.MAX_HAT_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.right_hat_y = event.state / Gamepad.MAX_HAT_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.right_hat_x = event.state / Gamepad.MAX_HAT_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.left_trigger = event.state / Gamepad.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.right_trigger = event.state / Gamepad.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.left_bumper = event.state
                elif event.code == 'BTN_TR':
                    self.right_bumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.a = event.state
                elif event.code == 'BTN_NORTH':
                    self.y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.x = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.b = event.state
                elif event.code == 'BTN_THUMBL':
                    self.left_thumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.right_thumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.back = event.state
                elif event.code == 'BTN_START':
                    self.start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.left_dpad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.right_dpad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.up_dpad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.down_dpad = event.state

    def update_robot_inputs(self, robot_ik: QuadropedIK, robot_gait: Gait):
        left_axis = np.array([self.left_hat_x, self.left_hat_y])
        right_axis = np.array([self.right_hat_x, self.right_hat_y])
        robot_ik.translation[0] = -0.06
        for axis in [left_axis, right_axis]:
            for i in range(len(axis)):
                if abs(axis[i]) < .2:
                    axis[i] = 0

        if self.right_trigger > .5:
            euler_angles = [
                np.interp(left_axis[0], [-1, 1], [-.8, .8]),
                np.interp(left_axis[1], [-1, 1], [-.8, .8]),
                np.interp(right_axis[0], [-1, 1], [-.8, .8])
            ]
            robot_ik.rotation = Rotation.from_euler("XYZ", euler_angles, degrees=False)
        elif self.left_trigger > .5:
            robot_ik.translation = np.array([
                np.interp(-left_axis[1], [-1, 1], [-.08, .08]) - 0.06,
                np.interp(left_axis[0], [-1, 1], [-.1, .1]),
                np.interp(right_axis[1], [-1, 1], [-.1, .1])
            ])
        else:
            velocity_frac = np.linalg.norm(left_axis)
            lateral_angle = math.atan2(left_axis[0], left_axis[1])
            robot_gait.target_speed = np.interp(velocity_frac, [-1, 1], [-.5, .5])
            robot_gait.lateral_rotation_angle = lateral_angle
            robot_gait.yaw_rate = np.interp(right_axis[0], [-1, 1], [-.5, .5])


