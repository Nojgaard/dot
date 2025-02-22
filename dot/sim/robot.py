from dm_control.composer import Entity, Observables
from dm_control.mujoco import Physics

# Composer high level imports
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control import mjcf
import numpy as np
from scipy.spatial.transform import Rotation

from dot.sim.rotation import quat_to_euler


class Robot(Entity):
    def __init__(self, model_path: str):
        self._model_path = model_path
        super().__init__()

    def _build(self):
        self._model = mjcf.from_path(self._model_path)
        self._imu = self._model.find("site", "imu")
        self.joint_ranges = np.array([j.range for j in self._model.find_all("joint")])

        for joint in self._model.find_all("joint"):
            if joint.type == "free":
                continue
            # joint.actuatorfrcrange = "-100 100"
            self._model.actuator.add(
                "position",
                name=joint.name,
                joint=joint,
                ctrlrange=joint.range,
                kp=15,
                kv=1,
            )

    def _build_observables(self):
        return RobotObservables(self)

    def angular_velocity(self, physics: Physics):
        return np.array(physics.bind(self.mjcf_model.sensor.gyro).sensordata)

    def linear_velocity(self, physics: Physics):
        velocimeter = self.mjcf_model.sensor.velocimeter
        return np.array(physics.bind(velocimeter).sensordata)

    def orientation(self, physics: Physics):
        framequat_element = self.mjcf_model.sensor.framequat
        quat = physics.bind(framequat_element).sensordata
        return quat_to_euler(quat)[:2]

    @property
    def observables(self) -> "RobotObservables":
        return self._observables

    @property
    def mjcf_model(self):
        return self._model

    @property
    def body_width(self) -> float:
        left_hip = self.mjcf_model.find("joint", "motor_front_left_shoulder")
        right_hip = self.mjcf_model.find("joint", "motor_front_right_shoulder")
        return np.linalg.norm(left_hip.pos - right_hip.pos)

    @property
    def body_length(self) -> float:
        front_hip = self.mjcf_model.find("joint", "motor_front_left_shoulder")
        rear_hip = self.mjcf_model.find("joint", "motor_rear_left_shoulder")
        return np.linalg.norm(front_hip.pos - rear_hip.pos)

    @property
    def max_height(self) -> float:
        return self.arm_length + self.wrist_length

    @property
    def arm_length(self) -> float:
        arm_joint = self._model.find("joint", "motor_front_left_arm")
        wrist_joint = self._model.find("joint", "motor_front_left_wrist")
        return np.linalg.norm(arm_joint.pos - wrist_joint.pos)

    @property
    def wrist_length(self) -> float:
        return 0.12

    @property
    def hip_offset(self) -> tuple[float, float]:
        shoulder_pos = self._model.find("joint", "motor_front_left_shoulder").pos
        arm_pos = self._model.find("joint", "motor_front_left_arm").pos
        shoulder_offsets = np.abs(shoulder_pos - arm_pos)

        return np.array([shoulder_offsets[2], shoulder_offsets[1]])

    @property
    def actuators(self):
        return tuple(self._model.find_all("actuator"))


class RobotObservables(Observables):
    @composer.observable
    def joint_positions(self):
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qpos", all_joints)

    @composer.observable
    def joint_velocities(self):
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qvel", all_joints)

    @composer.observable
    def attitude_velocity(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.gyro, index=[0, 1]
        )

    @composer.observable
    def accelerometer(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.accelerometer
        )

    def read_orientation(self, physics):
        framequat_element = self._entity.mjcf_model.sensor.framequat
        quat = physics.bind(framequat_element).sensordata
        return quat_to_euler(quat)[:2]

    @composer.observable
    def attitude(self):
        return observable.Generic(self.read_orientation)

    @composer.observable
    def sensors_framequat(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.framequat
        )
