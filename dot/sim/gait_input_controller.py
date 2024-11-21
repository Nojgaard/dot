from dataclasses import dataclass
from enum import Enum
from dot.control.gait import Gait
import numpy as np


class GaitMode(Enum):
    Forward = 0
    SideStep = 2
    Rotate = 1
    Complex = 3


@dataclass
class InputBounds:
    step_length = (0.02, 0.05)
    velocity = (0.05, 0.2)
    lateral = (-np.pi / 2.0, np.pi / 2.0)
    yaw = (-1, 1)


class GaitInputController:
    """Open loop controller simulating different control inputs an operator might take"""

    def __init__(self, gait: Gait, time_per_mode: float = 5):
        self.gait = gait
        self.time_per_mode = time_per_mode
        self.time = 0
        self.bounds = InputBounds()
        self.current_mode = GaitMode.Forward
        self._has_ramped_up = False

    def update(self, dt: float):
        self.time += dt

        if self.current_mode.value == 0 and self.time < 0.5:
            self.gait.step_length = 0
            self.gait.target_speed = 0.1
            self.gait.lateral_rotation_angle = 0
            self.gait.yaw_rate = 0
            return
        elif not self._has_ramped_up:
            self._has_ramped_up = True
            self.sample_control_inputs(self.current_mode)
            return

        if self.time < self.time_per_mode:
            return

        self.time = 0
        next_mode = self.current_mode.value + 1
        self.current_mode = GaitMode(min(GaitMode.Complex.value, next_mode))
        self.sample_control_inputs(self.current_mode)

    def initialize_episode(self):
        self.time = 0
        self.current_mode = GaitMode.Forward
        self._has_ramped_up = False
        self.sample_control_inputs(self.current_mode)

    def sample_control_inputs(self, mode: GaitMode):
        self.gait.step_length = 0.035
        self.gait.target_speed = 0.1
        self.gait.lateral_rotation_angle = 0
        self.gait.yaw_rate = 0

        rng = np.random.default_rng()
        bounds = self.bounds
        if mode in [GaitMode.Forward, GaitMode.Complex]:
            step_direction = 1 if rng.random() > 0.25 else -1
            self.gait.step_length = step_direction * rng.uniform(*bounds.step_length)
            self.gait.target_speed = rng.uniform(*bounds.velocity)

        if mode in [GaitMode.SideStep, GaitMode.Complex]:
            self.gait.lateral_rotation_angle = rng.uniform(*bounds.lateral)/2
            #self.gait.lateral_rotation_angle = np.clip(rng.normal(0, 0.5), *bounds.lateral)
            pass

        if mode in [GaitMode.Rotate, GaitMode.Complex]:
            self.gait.yaw_rate = rng.uniform(*bounds.yaw)/2
            #self.gait.yaw_rate = np.clip(rng.normal(0, 0.5), *bounds.yaw)

        #print("Mode: ", mode.name)
        #print("\tStep Length", self.gait.step_length)
        #print("\tVelocity", self.gait.target_speed)
        #print("\tLateral Angle", self.gait.lateral_rotation_angle)
        #print("\tYaw Rate", self.gait.yaw_rate)
