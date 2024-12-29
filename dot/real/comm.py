import asyncio
from dataclasses import dataclass
import asyncudp
import struct
import numpy as np
import copy

@dataclass
class SensorReadings:
    battery_voltage: float

class Comm:
    def __init__(self, socket: asyncudp.Socket):
        self._socket = socket
        self._is_connected = False
        self._sensor_readings = SensorReadings(-1)
        self._sensor_lock = asyncio.Lock()

    async def connect(self):
        print("LOCAL IP", self._local_ip())
        print(self._socket.getsockname())
        print("Connecting to robot", end="")
        while not self._is_connected:
            print(".", end="", flush=True)
            ip_packet = np.array([int(x) for x in self._local_ip().split(".")], dtype=np.uint8)
            self._socket.sendto(struct.pack("BBBB", *ip_packet))
            try:
                data, addr = await asyncio.wait_for(self._socket.recvfrom(), timeout=1)
            except asyncio.TimeoutError:
                continue

            print()
            print(f"Success! Linked {self._local_ip()} to {addr[0]}")
            print(f"Status: {ord(data)}")
            self._is_connected = True

    def send_packet(self, data: bytes):
        self._socket.sendto(data)

    async def listen_to_sensor_readings(self):
        while True:
            data, addr = await self._socket.recvfrom()
            unpacked_data = struct.unpack("f", data)
            async with self._sensor_lock:
                self._sensor_readings = SensorReadings(*unpacked_data)

    async def read_sensors(self):
        async with self._sensor_lock:
            readings = copy.copy(self._sensor_readings)
            return readings


    def _local_ip(self) -> str:
        return self._socket.getsockname()[0]
        local_ip = tuple(int(x) for x in local_ip_str.split("."))
        assert len(local_ip) == 4
        return local_ip

    @classmethod
    async def create(cls):
        socket = await asyncudp.create_socket(local_addr=("0.0.0.0", 9999), remote_addr=("10.0.0.88", 9999))
        return cls(socket)

