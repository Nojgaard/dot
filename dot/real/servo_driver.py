from dataclasses import dataclass
import struct
from dot.real.comm import Comm
import numpy as np
from numpy.typing import NDArray

class ServoDriver:
    def __init__(self, comm: Comm):
        self._comm = comm
        self._bounds_pwm_ms = (500, 2500)
        self._bounds_angle = (0, 180)

    def update_calibration(self, bounds_pwm_ms: tuple[int, int]):
        self._bounds_pwm_ms = tuple(bounds_pwm_ms)

    def set_angles(self, angles: list[int]):
        servo_pwm_ms = np.interp(angles, self._bounds_angle, self._bounds_pwm_ms)
        self._send_servo_pwm(servo_pwm_ms)

    def _send_servo_pwm(self, servo_pwm_ms: NDArray):
        assert(len(servo_pwm_ms) == 12)

        packet = np.asarray(np.round(servo_pwm_ms, decimals=0), dtype=np.int32)
        data = struct.pack("12i", *packet)
        self._comm.send_packet(data)

    