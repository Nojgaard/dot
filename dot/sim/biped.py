from dm_control.composer import Entity, Observables
from dm_control.mujoco import Physics

# Composer high level imports
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control import mjcf
import numpy as np
from scipy.spatial.transform import Rotation

from dot.sim.robot import Robot
from dot.sim.rotation import quat_to_euler


class Biped(Robot):
    def __init__(self):
        super().__init__(r"assets\model\biped.xml")

    @property
    def body_length(self) -> float:
        return 0