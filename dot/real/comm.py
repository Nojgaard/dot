import asyncio
import asyncudp
import struct
import numpy as np


class Comm:
    def __init__(self, socket: asyncudp.Socket):
        self._socket = socket
        self._is_connected = False

    async def connect(self):
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

    def _local_ip(self) -> str:
        return self._socket.getsockname()[0]
        local_ip = tuple(int(x) for x in local_ip_str.split("."))
        assert len(local_ip) == 4
        return local_ip

    @classmethod
    async def create(cls):
        socket = await asyncudp.create_socket(remote_addr=("10.0.0.88", 9999))
        return cls(socket)

