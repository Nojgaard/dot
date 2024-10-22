from dm_control.composer import Entity, Observables
from dm_control.mujoco import Physics
# Composer high level imports
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control import mjcf
import numpy as np


class Spot(Entity):
    def _build(self):
        #self._model = mjcf.from_path(r"assets\model\xml\spot_mini\spot_mini.xml")
        self._model = mjcf.from_path(r"assets\model\spot.xml")
        

        for joint in self._model.find_all("joint"):
            if joint.type == "free":
                continue
            #joint.actuatorfrcrange = "-100 100"
            self._model.actuator.add('position', name=joint.name, joint=joint, kp=15, kv=1)

    def _build_observables(self):
        return SpotObservables(self)
    
    @property
    def observables(self) -> "SpotObservables":
        return self._observables

    @property
    def mjcf_model(self):
        return self._model
    
    @property
    def body_width(self) -> float:
        left_hip = self.mjcf_model.find("body", "front_left_shoulder_link")
        right_hip = self.mjcf_model.find("body", "front_right_shoulder_link")
        return np.linalg.norm(left_hip.pos - right_hip.pos)
    
    @property
    def body_length(self) -> float:
        front_hip = self.mjcf_model.find("body", "front_left_shoulder_link")
        rear_hip = self.mjcf_model.find("body", "rear_left_shoulder_link")
        return np.linalg.norm(front_hip.pos - rear_hip.pos)
    
    @property
    def max_height(self) -> float:
        return self.shoulder_length + self.wrist_length
    
    @property
    def shoulder_length(self) -> float:
        body_elbow = self._model.find("body", "front_left_foot_link")
        return np.linalg.norm(body_elbow.pos)
    
    @property
    def wrist_length(self) -> float:
        geom_foot = self._model.find("geom", "front_left_foot_mesh")
        return np.linalg.norm(geom_foot.pos)
    
    @property
    def hip_offset(self) -> tuple[float, float]:
        body_shoulder = self._model.find("body", "front_left_leg_link")
        return np.abs(np.array(body_shoulder.pos[1:][::-1]))

    @property
    def actuators(self):
        return tuple(self._model.find_all("actuator"))


class SpotObservables(Observables):
    @composer.observable
    def joint_positions(self):
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qpos", all_joints)

    @composer.observable
    def joint_velocities(self):
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qvel", all_joints)
