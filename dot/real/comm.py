import asyncio
from dataclasses import dataclass
import asyncudp
import struct
import numpy as np
import copy

from dot.real import packet

class Comm:
    def __init__(self, socket: asyncudp.Socket):
        self._socket = socket
        self._is_connected = False
        self._telemetry = packet.Telemetry.default()
        self._sensor_lock = asyncio.Lock()

    async def connect(self):
        print("LOCAL IP", self._local_ip())
        print(self._socket.getsockname())
        print("Connecting to robot", end="")
        while not self._is_connected:
            print(".", end="", flush=True)
            self._socket.sendto(packet.build_ping())
            try:
                data, addr = await asyncio.wait_for(self._socket.recvfrom(), timeout=1)
            except asyncio.TimeoutError:
                continue

            print()
            print(f"Success! Linked {self._local_ip()} to {addr[0]}")
            #print(f"Status: {ord(data)}")
            self._is_connected = True

    def send_packet(self, data: bytes):
        self._socket.sendto(data)

    async def listen_to_sensor_readings(self):
        while True:
            data, addr = await self._socket.recvfrom()
            telemetry = packet.Telemetry.from_packet(data)
            if telemetry is None:
                print("Telemetry packet is of unexpected format")
                continue
            
            self._telemetry = telemetry

    @property
    def telemetry(self):
        return self._telemetry

    def _local_ip(self) -> str:
        return self._socket.getsockname()[0]

    @classmethod
    async def create(cls):
        socket = await asyncudp.create_socket(local_addr=("0.0.0.0", 9999), remote_addr=("10.0.0.88", 9999))
        return cls(socket)

