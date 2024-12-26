from dataclasses import dataclass
import math
import struct
from dot.real.comm import Comm
import numpy as np
from numpy.typing import NDArray


class ServoDriver:
    def __init__(self, comm: Comm):
        self._comm = comm
        self._bounds_pwm_ms = (500, 2500)
        self._bounds_servo_angle = (0, 180)

    def update_calibration(self, bounds_pwm_ms: tuple[int, int]):
        self._bounds_pwm_ms = tuple(bounds_pwm_ms)

    def set_servo_angles(self, servo_angles: NDArray[np.floating]):
        servo_pwm_ms = np.interp(
            servo_angles, self._bounds_servo_angle, self._bounds_pwm_ms
        )
        self._send_servo_pwm(servo_pwm_ms)

    def set_joint_angles(
        self, joint_angles: NDArray[np.floating], joint_ranges: NDArray[np.floating]
    ):
        servo_angles = self.joint_to_servo_angle(joint_angles, joint_ranges)
        self.set_servo_angles(servo_angles)

    def joint_to_servo_angle(
        self, joint_angles: NDArray[np.floating], joint_ranges: NDArray[np.floating]
    ) -> NDArray[np.floating]:
        nominal_angle = np.array(
            [np.interp(0, jrng, self._bounds_servo_angle) for jrng in joint_ranges]
        )
        nominal_angle += np.rad2deg(joint_angles)
        return nominal_angle


    def _send_servo_pwm(self, servo_pwm_ms: NDArray):
        assert len(servo_pwm_ms) == 12

        packet = np.asarray(np.round(servo_pwm_ms, decimals=0), dtype=np.int32)
        data = struct.pack("12i", *packet)
        self._comm.send_packet(data)
