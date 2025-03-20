from dataclasses import dataclass
from enum import Enum
import struct
import numpy as np
from numpy.typing import NDArray


class PacketType(Enum):
    SERVO_MS = 1
    SERVO_TARGET_ANGLE = 2
    SERVO_CALIBRATION = 3
    PING = 4

class ImuStatus(Enum):
    UNDEFINED = -1
    OK = 0
    MEM_ERROR = 1
    DMP_ERROR = 2
    I2C_ERROR = 3

class ServoStatus(Enum):
    UNDEFINED = -1
    OK = 0
    I2C_ERROR = 1
    MISSING_CALIBRATION = 2

@dataclass
class Telemetry:
    battery_voltage: float
    battery_current: float
    orientation: NDArray[np.floating]
    acceleration: NDArray[np.floating]
    imu_status: ImuStatus
    servo_status: ServoStatus

    @staticmethod
    def default():
        return Telemetry(
            battery_voltage=-1,
            battery_current=-1,
            orientation=np.array([0.0, 0.0, 0.0]),
            acceleration=np.array([0.0, 0.0, 0.0]),
            imu_status=ImuStatus(-1),
            servo_status=ServoStatus(-1),
        )

    @staticmethod
    def from_packet(data: bytes):
        packet_format = "8f4b"
        if struct.calcsize(packet_format) != len(data):
            return None

        packet = struct.unpack(packet_format, data)
        return Telemetry(
            battery_voltage=packet[0],
            battery_current=packet[1],
            orientation=np.array(packet[2:5]),
            acceleration=np.array(packet[5:8]),
            imu_status=ImuStatus(packet[8]),
            servo_status=ServoStatus(packet[9]),
        )


def build_servo_ms_packet(microseconds: list[int]) -> bytes:
    packet_type = PacketType.SERVO_MS.value
    data = struct.pack("b12i", packet_type, *microseconds)
    return data


def servo_target_angle_packet(target_angles: list[float]) -> bytes:
    packet_type = PacketType.SERVO_TARGET_ANGLE.value
    data = struct.pack("b12f", packet_type, *target_angles)
    return data


def servo_calibration_packet(
    degreesPerSecond: float,
    smoothingFactor: float,
    min_microseconds: list[int],
    max_microseconds: list[int],
    max_servo_angles: list[int],
):
    packet_type = PacketType.SERVO_CALIBRATION.value
    data = struct.pack(
        "bff36i",
        packet_type,
        degreesPerSecond,
        smoothingFactor,
        *min_microseconds,
        *max_microseconds,
        *max_servo_angles
    )
    return data


def build_ping() -> bytes:
    return struct.pack("b", PacketType.PING.value)
