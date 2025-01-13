from dm_control import composer
from dm_control.composer import Entity, Task
from dm_control.composer.observation import observable
from dm_control.composer.variation import distributions, noises
from dm_control.locomotion.arenas import floors, Bowl
from dm_control.mujoco import Physics
import dm_env.specs

from dot.control.gait import Gait
from dot.control.inverse_kinematics import RobotIK
from dot.sim.bumpy_arena import BumpyArena
from dot.sim.gait_input_controller import GaitInputController
from dot.sim.quadruped import Quadruped
from scipy.spatial.transform import Rotation
import numpy as np
from numpy.typing import NDArray

import math
from dot.sim.rotation import quat_to_euler
from dm_control.utils import rewards


class ModulateGaitTask(Task):
    def __init__(
        self,
        model: Quadruped,
        model_ik: RobotIK,
        model_gait: Gait,
        input_controller: GaitInputController,
        arena: composer.Arena = floors.Floor(reflectance=0.0)
    ) -> None:
        self.model = model
        self.model_ik = model_ik
        self.model_gait = model_gait
        self.input_controller = input_controller
        self.enable_input_controller = False

        self._rest_joint_angles = model_ik.find_angles().flatten()[
            : len(model.actuators)
        ]
        
        self._arena = arena
        #self._arena = BumpyArena()
        #self._arena = floors.Floor(reflectance=0.0)
        #self._creature_initial_pose = (0, 0, 0.65)
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
        self._last_actions: list[NDArray] = [
            self._rest_joint_angles.copy(),
            self._rest_joint_angles.copy(),
            self._rest_joint_angles.copy(),
        ]

        self.model.observables.attitude_velocity.enabled = True
        self.model.observables.attitude.enabled = True
        self._task_observables = {
            "leg_phases": observable.Generic(
                lambda physics: self.model_gait.leg_phase()[:1]
            ),
            "prev_actions": observable.Generic(
                lambda physics: np.array(self._last_actions)[:2].flatten()
            )
        }

        for obs in self._task_observables.values():
            obs.enabled = True

    @property
    def root_entity(self):
        return self._arena

    @property
    def task_observables(self):
        return self._task_observables

    def action_spec(self, physics):
        action_shape = (len(self.model.actuators),)
        joint_bias_range = (-0.1, 0.1)

        minimum = np.zeros(action_shape)
        minimum[:] = joint_bias_range[0]

        maximum = np.zeros(action_shape)
        maximum[:] = joint_bias_range[1]

        return dm_env.specs.BoundedArray(
            shape=action_shape,
            dtype=np.float32,
            minimum=minimum,
            maximum=maximum,
            name="modulate_gait",
        )

    def _compute_joint_angles(self, dt: float):
        foot_positions = self.model_gait.compute_foot_positions(dt)
        joint_angles = self.model_ik.find_angles(foot_positions)
        return joint_angles.flatten()[: len(self.model.actuators)]

    def before_step(self, physics, action, random_state):
        dt = self.control_timestep
        foot_positions = self.model_gait.compute_foot_positions(dt)
        joint_angles = self.model_ik.find_angles(foot_positions)
        joint_angles = joint_angles.flatten()[: len(self.model.actuators)]
        joint_angles += action

        joint_angles = np.clip(joint_angles, self.model.joint_ranges[:, 0], self.model.joint_ranges[:, 1])
        self._last_actions = [
            joint_angles,
            self._last_actions[0],
            self._last_actions[1],
        ]
        self._last_bias = action
        super().before_step(physics, joint_angles, random_state)

    def after_step(self, phsyics, random_state):
        if self.enable_input_controller:
            dt = self.control_timestep
            self.input_controller.update(dt)

    def initialize_episode(self, physics: Physics, random_state):
        self.model.set_pose(
            physics, self._creature_initial_pose, np.array([1, 0, 0, 0])
        )
        joint_names = [j.name for j in self.model.mjcf_model.find_all("joint")]
        for name, angle in zip(joint_names, self._rest_joint_angles):
            physics.named.data.qpos[f"spot/{name}"] = angle

        if self.enable_input_controller:
            self.input_controller.initialize_episode()

        self._arena.regenerate(random_state)
        self._arena.initialize_episode(physics, random_state)

    def get_reward(self, physics: Physics) -> float:
        attitude = self.model.orientation(physics)[:2]
        linear_velocity = self.model.linear_velocity(physics)
        angular_velocity = self.model.angular_velocity(physics)
        actions = self._last_actions

        joints = self.model.mjcf_model.find_all("joint")
        joint_forces = physics.data.actuator_force
        joint_accelerations = physics.bind(joints).qacc

        target_yaw = self.model_gait.yaw_rate
        lateral_angle = self.model_gait.lateral_rotation_angle
        xv_frac, yv_frac = np.cos(lateral_angle), np.sin(lateral_angle)
        target_velocity = np.array(
            xv_frac * self.model_gait.target_speed,
            yv_frac * self.model_gait.target_speed,
        )

        rwd_torso_attitude = math.exp(-20 * np.linalg.norm(attitude))
        rwd_linear_xy_velocity = math.exp(
            -8 * np.linalg.norm(target_velocity - linear_velocity[:2])
        )
        rwd_linear_z_velocity = math.exp(-8 * linear_velocity[2] ** 2)
        rwd_angular_xy_velocity = math.exp(-2 * np.linalg.norm(angular_velocity[:2]))
        rwd_angular_z_velocity = math.exp(-2 * (target_yaw - angular_velocity[2]) ** 2)

        rwd_joint_torques = -np.linalg.norm(joint_forces)
        rwd_joint_accelerations = -np.linalg.norm(joint_accelerations)
        rwd_leg_action_rate = -np.linalg.norm(actions[0] - actions[1])
        rwd_leg_action_acc = -np.linalg.norm(actions[0] - 2 * actions[1] + actions[2])

        rwd_survival = 5

        reward = (
            1 * rwd_torso_attitude
            + 1 * rwd_linear_xy_velocity
            + 1 * rwd_linear_z_velocity
            + 0.5 * rwd_angular_xy_velocity
            + 0.5 * rwd_angular_z_velocity
            + 1e-3 * rwd_joint_torques
            + 2.5e-6 * rwd_joint_accelerations
            + 1.5 * rwd_leg_action_rate
            + 0.45 * rwd_leg_action_acc
            + rwd_survival
        )
        return reward

    def get_reward_old(self, physics: Physics) -> float:
        attitude = self.model.orientation(physics)[:2]
        linear_velocity = self.model.linear_velocity(physics)
        angular_velocity = self.model.angular_velocity(physics)
        lateral_angle = self.model_gait.lateral_rotation_angle
        xv_frac, yv_frac = np.cos(lateral_angle), np.sin(lateral_angle)

        target_xv = self.model_gait.target_speed * xv_frac
        forward_reward = rewards.tolerance(
            linear_velocity[0],
            bounds=(target_xv, target_xv),
            margin=1,
            sigmoid="linear",
        )

        target_yv = self.model_gait.target_speed * yv_frac
        lateral_reward = rewards.tolerance(
            linear_velocity[1],
            bounds=(target_yv, target_yv),
            margin=1,
            sigmoid="linear",
        )

        stability_reward = rewards.tolerance(
            attitude, bounds=(0, 0), margin=math.radians(60), sigmoid="linear"
        )

        joints = self.model.mjcf_model.find_all("joint")
        joint_forces = np.abs(physics.data.actuator_force)
        joint_velocities = np.abs(physics.bind(joints).qvel)
        # print((joint_forces * joint_velocities).mean())
        energy_reward = rewards.tolerance(
            # np.dot(joint_forces, joint_velocities),
            (joint_forces * joint_velocities).mean(),
            bounds=(0, 0),
            margin=2,
            sigmoid="gaussian",
        )
        energy_reward = (1 + 4 * energy_reward) / 5
        # print(small_action_reward)

        reward_terms = np.array(
            [
                forward_reward,
                lateral_reward,
                energy_reward,
                *stability_reward,
            ]
        )
        # print(np.prod(reward_terms))

        return np.prod(reward_terms)

    def should_terminate_episode(self, physics):
        pose, quat = self.model.get_pose(physics)
        orientation = np.abs(quat_to_euler(quat))
        thres_angle = math.radians(60)
        has_fallen = orientation[0] > thres_angle or orientation[1] > thres_angle
        has_fallen = has_fallen or (pose[2] < 0.08)
        return has_fallen
