from dataclasses import dataclass
from enum import Enum
from dot.control.gait import Gait
import numpy as np


class GaitMode(Enum):
    RampUp = 0
    Forward = 1
    Rotate = 2
    SideStep = 3
    Complex = 4


@dataclass
class InputBounds:
    velocity = (0.1, 0.5)
    lateral = (-np.pi / 8.0, np.pi / 8.0)
    yaw = (-0.2, 0.2)


class GaitInputController:
    """Open loop controller simulating different control inputs an operator might take"""

    def __init__(self, gait: Gait, time_per_mode: float = 5):
        self.gait = gait
        self.time_per_mode = time_per_mode
        self.time = 0
        self.bounds = InputBounds()

        self.mode_sequence = np.arange(len(GaitMode))
        np.random.shuffle(self.mode_sequence[1:])
        self.current_mode_idx = 0

    def update(self, dt: float):
        self.time += dt
        time_per_mode = self.time_per_mode if self.current_mode_idx > 0 else 0.2

        if self.time < time_per_mode:
            return

        self.time = 0
        self.current_mode_idx = min(len(GaitMode) - 1, self.current_mode_idx + 1)
        self.sample_control_inputs()

    def initialize_episode(self):
        self.time = 0
        self.current_mode_idx = 0
        np.random.shuffle(self.mode_sequence[1:])
        self.sample_control_inputs()

    def sample_control_inputs(self):
        self.gait.target_speed = 0
        self.gait.lateral_rotation_angle = 0
        self.gait.yaw_rate = 0

        mode = GaitMode(self.mode_sequence[self.current_mode_idx])
        rng = np.random.default_rng()
        bounds = self.bounds

        if mode == GaitMode.RampUp:
            return
        elif mode == GaitMode.Forward:
            direction = 1 if rng.random() > 0.20 else -1
            self.gait.target_speed = direction * rng.uniform(*bounds.velocity)
        elif mode == GaitMode.Rotate:
            self.gait.target_speed = 0.3
            self.gait.yaw_rate = rng.uniform(*bounds.yaw)
        elif mode == GaitMode.SideStep:
            self.gait.target_speed = 0.3
            self.gait.lateral_rotation_angle = rng.uniform(*bounds.lateral)
        elif mode == GaitMode.Complex:
            direction = 1 if rng.random() > 0.20 else -1
            self.gait.target_speed = direction * rng.uniform(*bounds.velocity)
            self.gait.yaw_rate = rng.uniform(*bounds.yaw)
            self.gait.lateral_rotation_angle = rng.uniform(*bounds.lateral)
        else:
            raise NotImplementedError()

        # print("Mode: ", mode.name)
        # print("\tStep Length", self.gait.step_length)
        # print("\tVelocity", self.gait.target_speed)
        # print("\tLateral Angle", self.gait.lateral_rotation_angle)
        # print("\tYaw Rate", self.gait.yaw_rate)
