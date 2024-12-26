import asyncio
from enum import Enum
from typing import Callable, Self
import numpy as np
import time

from dot.real.comm import Comm
from dot.control.inverse_kinematics import QuadropedIK
from dot.control.gait import Gait
from dot.real.servo_driver import ServoDriver
from dot.sim.quadruped import Quadruped


class RobotController:
    class Mode(Enum):
        Calibrate = 0

    def __init__(self, comm: Comm):
        self._comm = comm
        self.servo_driver = ServoDriver(self._comm)

        self.robot_specs = Quadruped()
        self.robot_ik = QuadropedIK(
            self.robot_specs.body_length,
            self.robot_specs.body_width,
            self.robot_specs.max_height * 0.7,
            self.robot_specs.hip_offset,
            self.robot_specs.shoulder_length,
            self.robot_specs.wrist_length,
            translation=np.array([-0.035, 0, 0]),
        )
        self.robot_gait = Gait(self.robot_ik.foot_points)

    async def launch(self, mode: Mode, callback: Callable[[Self], None], fps: int = 30):
        frame_interval = 1.0 / fps
        await self._comm.connect()
        prev_time = time.time()
        while True:
            current_time = time.time()
            dt = current_time - prev_time
            callback()
            # work....i

            elapsed_frame_time = time.time() - current_time
            await asyncio.to_thread(
                lambda: time.sleep(max(0, frame_interval - elapsed_frame_time))
            )
            prev_time = current_time

    @classmethod
    async def create(cls):
        comm = await Comm.create()
        return cls(comm)


async def main():
    controller = await RobotController.create()
    await controller.launch()


if __name__ == "__main__":
    asyncio.run(main())
