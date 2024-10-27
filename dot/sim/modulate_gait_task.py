from dm_control import composer
from dm_control.composer import Entity, Task
from dm_control.composer.observation import observable
from dm_control.composer.variation import distributions, noises
from dm_control.locomotion.arenas import floors
from dm_control.mujoco import Physics

from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.quadruped import Quadruped
from scipy.spatial.transform import Rotation
import numpy as np
from numpy.typing import NDArray


class ModulateGaitTask(Task):
    def __init__(self, model: Quadruped, model_ik: QuadropedIK, model_gait: Gait) -> None:
        self.model = model
        self.model_ik = model_ik
        self.model_gait = model_gait

        self._rest_joint_angles = model_ik.find_angles().flatten()

        self._arena = floors.Floor(reflectance=0.0)
        self._creature_initial_pose = (0, 0, 0.26)
        self._arena.add_free_entity(self.model)
        #self._arena.mjcf_model.worldbody.add('light', pos=(0, 0, 4))
        self.set_timesteps(control_timestep=0.03, physics_timestep=0.005)

        # Configure and enable observables
        #pos_corrptor = noises.Additive(distributions.Normal(scale=0.01))
        #self._spot.observables.joint_positions.corruptor = pos_corrptor
        #self._spot.observables.joint_positions.enabled = True
        #vel_corruptor = noises.Multiplicative(distributions.LogNormal(sigma=0.01))
        #self._spot.observables.joint_velocities.corruptor = vel_corruptor
        #self._spot.observables.joint_velocities.enabled = True

        self.model.observables.sensors_framequat.enabled = True
        #self.model.observables.sensors_gyro.enabled = True
        self.model.observables.sensors_orientation.enabled = True
        self._task_observables = {}
        self._last_position: NDArray = np.zeros(3)
    
    @property
    def root_entity(self):
        return self._arena

    @property
    def task_observables(self):
        return self._task_observables
    
    def action_spec(self, physics):
        return super().action_spec(physics)
    
    def before_step(self, physics, action, random_state):
        dt = self.control_timestep
        foot_positions = self.model_gait.compute_foot_positions(dt)
        joint_angles = self.model_ik.find_angles(foot_positions)
        super().before_step(physics, joint_angles.flatten(), random_state)
    
    def initialize_episode(self, physics: Physics, random_state):
        self.model.set_pose(physics, self._creature_initial_pose, np.array([1, 0, 0, 0]))
        joint_names = [j.name for j in self.model.mjcf_model.find_all("joint")]
        for name, angle in zip(joint_names, self._rest_joint_angles):
           physics.named.data.qpos[f"spot/{name}"] = angle

        self._last_position = self.model.position(physics)

    
    def get_reward(self, physics: Physics) -> float:
        position = self.model.position(physics)
        orientation = self.model.orientation(physics)
        angular_velocity = self.model.angular_velocity(physics)
        dpos = position - self._last_position

        distance_weight = 1.0
        drift_weight = 2.0
        orientation_weight = 5.0
        angular_velocity_weight = 0.05

        reward = distance_weight * dpos[0]
        reward -= drift_weight * position[1]
        reward -= orientation_weight * np.sum(np.abs(orientation[:2]))
        reward -= angular_velocity_weight * np.sum(np.abs(angular_velocity[:2]))
        self._last_position = position
        return reward

        