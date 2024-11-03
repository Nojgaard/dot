from dm_control import composer
from dm_control.composer import Entity, Task
from dm_control.composer.observation import observable
from dm_control.composer.variation import distributions, noises
from dm_control.locomotion.arenas import floors
from dm_control.mujoco import Physics
import dm_env.specs

from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.quadruped import Quadruped
from scipy.spatial.transform import Rotation
import numpy as np
from numpy.typing import NDArray

import math
from dot.sim.rotation import quat_to_euler


class ModulateGaitTask(Task):
    def __init__(
        self, model: Quadruped, model_ik: QuadropedIK, model_gait: Gait
    ) -> None:
        self.model = model
        self.model_ik = model_ik
        self.model_gait = model_gait

        self._rest_joint_angles = model_ik.find_angles().flatten()

        self._arena = floors.Floor(reflectance=0.0)
        self._creature_initial_pose = (0, 0, 0.19)
        self._arena.add_free_entity(self.model)
        # self._arena.mjcf_model.worldbody.add('light', pos=(0, 0, 4))
        self.set_timesteps(control_timestep=0.03, physics_timestep=0.005)

        # Configure and enable observables
        # pos_corrptor = noises.Additive(distributions.Normal(scale=0.01))
        # self._spot.observables.joint_positions.corruptor = pos_corrptor
        # self._spot.observables.joint_positions.enabled = True
        # vel_corruptor = noises.Multiplicative(distributions.LogNormal(sigma=0.01))
        # self._spot.observables.joint_velocities.corruptor = vel_corruptor
        # self._spot.observables.joint_velocities.enabled = True

        # self.model.observables.sensors_framequat.enabled = True
        self.model.observables.attitude_velocity.enabled = True
        self.model.observables.attitude.enabled = True
        self._task_observables = {
            "leg_phases": observable.Generic(
                lambda physics: self.model_gait.leg_phase()
            )
        }

        for obs in self._task_observables.values():
            obs.enabled = True
        self._last_position: NDArray = np.zeros(3)

    @property
    def root_entity(self):
        return self._arena

    @property
    def task_observables(self):
        return self._task_observables

    def action_spec(self, physics):
        action_shape = (14,)
        #curve_z_offset_range = (0, 0.15)
        curve_z_offset_range = (0, 0.1)
        foot_bias_range = (-0.1, 0.1)

        minimum = np.zeros(action_shape)
        minimum[:2] = curve_z_offset_range[0]
        minimum[2:] = foot_bias_range[0]

        maximum = np.zeros(action_shape)
        maximum[:2] = curve_z_offset_range[1]
        maximum[2:] = foot_bias_range[1]

        return dm_env.specs.BoundedArray(
            shape=action_shape, 
            dtype=np.float64, 
            minimum=minimum,
            maximum=maximum,
            name="modulate_gait"
        )

    def before_step(self, physics, action, random_state):
        self.model_gait.penetration_depth = action[0]
        self.model_gait.clearance_height = action[1]

        foot_bias = action[2:].reshape((4, 3))

        dt = self.control_timestep
        foot_positions = self.model_gait.compute_foot_positions(dt)
        foot_positions += foot_bias

        joint_angles = self.model_ik.find_angles(foot_positions)
        super().before_step(physics, joint_angles.flatten(), random_state)

    def initialize_episode(self, physics: Physics, random_state):
        self.model.set_pose(
            physics, self._creature_initial_pose, np.array([1, 0, 0, 0])
        )
        joint_names = [j.name for j in self.model.mjcf_model.find_all("joint")]
        for name, angle in zip(joint_names, self._rest_joint_angles):
            physics.named.data.qpos[f"spot/{name}"] = angle

        self._last_position = np.array(self.model.get_pose(physics)[0])

    def get_reward(self, physics: Physics) -> float:
        position, quat = self.model.get_pose(physics)
        orientation = quat_to_euler(quat)
        linear_velocity, angular_velocity = self.model.get_velocity(physics)
        #dpos = position - self._last_position

        distance_weight = 1.0#1
        drift_weight = 0.5#2.0
        orientation_weight = 2.0#5.0
        #angular_velocity_weight = 0.05

        reward = distance_weight * linear_velocity[0]
        reward -= drift_weight * abs(linear_velocity[1])
        #reward -= drift_weight * abs(position[1])
        #reward -= drift_weight * abs(position[1])
        reward -= orientation_weight * np.sum(np.abs(orientation[:2]))

        thres_angle = math.radians(60)
        has_fallen = abs(orientation[0]) > thres_angle or abs(orientation[1]) > thres_angle
        #if has_fallen:
        #    reward -= 1000 
        #reward -= angular_velocity_weight * np.sum(np.abs(angular_velocity[:2]))
        self._last_position = np.array(position)
        #print(dpos[0], np.sum(np.abs(orientation[:2])), abs(position[1]))
        return reward
    
    def should_terminate_episode(self, physics):
        _, quat = self.model.get_pose(physics)
        orientation = np.abs(quat_to_euler(quat))
        thres_angle = math.radians(60)
        has_fallen = orientation[0] > thres_angle or orientation[1] > thres_angle
        return has_fallen