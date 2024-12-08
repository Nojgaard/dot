import asyncio
import asyncudp


class Comm:
    def __init__(self, socket: asyncudp.Socket):
        self._socket = socket
        self._is_connected = False

    async def connect(self):
        print("Connecting to robot", end="")
        while not self._is_connected:
            print(".", end="", flush=True)
            self._socket.sendto(self._local_ip().encode())
            try:
                data, addr = await asyncio.wait_for(self._socket.recvfrom(), timeout=1)
            except asyncio.TimeoutError:
                continue

            print()
            print("Success!")
            print(f"Received {data} from {addr}")
            self._is_connected = True

    def _local_ip(self) -> str:
        return self._socket.getsockname()[0]

    @classmethod
    async def create(cls):
        socket = await asyncudp.create_socket(remote_addr=("10.0.0.17", 9999))
        return cls(socket)
    
async def main():
    comm = await Comm.create()
    await comm.connect()


asyncio.run(main())
