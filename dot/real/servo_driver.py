from dataclasses import dataclass
import math
import struct
from dot.real import packet
from dot.real.comm import Comm
import numpy as np
from numpy.typing import NDArray
import json


class ServoDriver:
    @dataclass
    class Calibration:
        num_servos = 12
        degreesPerSecond = 180
        bounds_pwm_ms = [(500, 2500) for _ in range(12)]
        bounds_servo_angle = (0, 180)
        invert_servo_angle = [False] * 12
        servo_angle_bias = [0] * 12

    def __init__(self, comm: Comm):
        self._comm = comm
        self.calibration = ServoDriver.Calibration()

    def set_servo_angles(
        self, servo_angles: NDArray[np.floating], send_pwm_directly: bool = False
    ):
        if not send_pwm_directly:
            self._send_target_servo_angles(servo_angles)
            return

        servo_pwm_ms = [
            np.interp(
                angle,
                self.calibration.bounds_servo_angle,
                pwm_bounds,
            )
            for angle, pwm_bounds in zip(servo_angles, self.calibration.bounds_pwm_ms)
        ]
        self._send_servo_pwm(servo_pwm_ms)

    def set_joint_angles(
        self,
        joint_angles: NDArray[np.floating],
        joint_ranges: NDArray[np.floating],
    ):
        servo_angles = self.joint_to_servo_angle(joint_angles, joint_ranges)
        self.set_servo_angles(servo_angles)

    def joint_to_servo_angle(
        self,
        joint_angles: NDArray[np.floating],
        joint_ranges: NDArray[np.floating],
    ) -> NDArray[np.floating]:
        bounds_servo_angle = self.calibration.bounds_servo_angle
        servo_angle_bias = self.calibration.servo_angle_bias
        invert_servo = self.calibration.invert_servo_angle

        nominal_angle = np.array(
            [np.interp(0, jrng, bounds_servo_angle) for jrng in joint_ranges]
        )
        nominal_angle += np.rad2deg(joint_angles)
        nominal_angle = np.array(
            [
                bounds_servo_angle[1] - x if i else x
                for x, i in zip(nominal_angle, invert_servo)
            ]
        )
        return nominal_angle + servo_angle_bias

    def _send_servo_pwm(self, servo_pwm_ms: NDArray):
        assert len(servo_pwm_ms) == 12

        bounds_pwn = self.calibration.bounds_pwm_ms
        microseconds = np.asarray(np.round(servo_pwm_ms, decimals=0), dtype=np.int32)
        microseconds = np.clip(
            microseconds, [x[0] for x in bounds_pwn], [x[1] for x in bounds_pwn]
        )
        data = packet.build_servo_ms_packet(microseconds)
        self._comm.send_packet(data)

    def _send_target_servo_angles(self, target_angles: NDArray[np.floating]):
        target_angles = np.clip(target_angles, *self.calibration.bounds_servo_angle)
        data = packet.servo_target_angle_packet(target_angles)
        self._comm.send_packet(data)

    def send_calibration(self):
        min_microseconds, max_microseconds = zip(*self.calibration.bounds_pwm_ms)
        max_servo_angles = self.calibration.num_servos * [
            self.calibration.bounds_servo_angle[1]
        ]
        data = packet.servo_calibration_packet(
            self.calibration.degreesPerSecond,
            min_microseconds,
            max_microseconds,
            max_servo_angles,
        )
        self._comm.send_packet(data)

    def load_calibration(self, path: str):
        with open(path) as f:
            jdata = json.load(f)

        calibration = self.calibration
        for jservo in jdata["servos"]:
            servo_id = jservo["id"]
            calibration.invert_servo_angle[servo_id] = jservo["invert"]
            calibration.bounds_pwm_ms[servo_id] = jservo["pwm_range"]
            calibration.servo_angle_bias[servo_id] = jservo["degree_bias"]
