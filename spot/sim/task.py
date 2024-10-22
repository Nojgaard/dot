from dm_control.composer import Entity, Task
from dm_control.mujoco import Physics
# Composer high level imports
from dm_control import composer
from dm_control.locomotion.arenas import floors
from dm_control.composer.observation import observable
from dm_control.composer.variation import distributions
from dm_control.composer.variation import noises

from spot.sim import spot

class WalkTask(Task):
    def __init__(self, model: spot.Spot, initial_angles) -> None:
        self._model = model
        self._initial_angles = initial_angles

        self._arena = floors.Floor(reflectance=0.0)
        self._creature_initial_pose = (0, 0, 0.26)
        self._arena.add_free_entity(self._model)
        #self._arena.mjcf_model.worldbody.add('light', pos=(0, 0, 4))
        #self._arena.mjcf_model.option.timestep = "0.005"
        self.set_timesteps(control_timestep=0.03, physics_timestep=0.005)

        # Configure and enable observables
        #pos_corrptor = noises.Additive(distributions.Normal(scale=0.01))
        #self._spot.observables.joint_positions.corruptor = pos_corrptor
        #self._spot.observables.joint_positions.enabled = True
        #vel_corruptor = noises.Multiplicative(distributions.LogNormal(sigma=0.01))
        #self._spot.observables.joint_velocities.corruptor = vel_corruptor
        #self._spot.observables.joint_velocities.enabled = True

        self._task_observables = {}
    
    @property
    def root_entity(self):
        return self._arena

    @property
    def task_observables(self):
        return self._task_observables
    
    def initialize_episode(self, physics: Physics, random_state):
        self._model.set_pose(physics, self._creature_initial_pose)
        joint_names = [j.name for j in self._model.mjcf_model.find_all("joint")]
        for name, angle in zip(joint_names, self._initial_angles):
           physics.named.data.qpos[f"spot/{name}"] = angle

    
    def get_reward(self, physics) -> float:
        return 0

        