import asyncio
from enum import Enum
from typing import Callable, Self
import numpy as np
import time

from dot.real import packet
from dot.real.comm import Comm
from dot.control.inverse_kinematics import RobotIK
from dot.control.gait import Gait
from dot.real.servo_driver import ServoDriver
from dot.sim.quadruped import Quadruped


class RobotController:
    class Mode(Enum):
        Calibrate = 0
        Normal = 1

    def __init__(self, comm: Comm):
        self._comm = comm
        self.servo_driver = ServoDriver(self._comm)

        self.robot_specs = Quadruped()
        self.robot_ik = RobotIK(
            self.robot_specs.body_length,
            self.robot_specs.body_width,
            self.robot_specs.max_height * 0.7,
            self.robot_specs.hip_offset,
            # 0.115,
            # 0.135,
            self.robot_specs.arm_length,
            self.robot_specs.wrist_length,
            translation=np.array([-0.04, 0, 0]),
        )
        self.robot_gait = Gait(self.robot_ik.foot_points)

    async def sync_with_firmware(self):
        needed_to_sync = False
        servo_status = self._comm.telemetry.servo_status

        if servo_status != packet.ServoStatus.MISSING_CALIBRATION:
            return needed_to_sync

        print("Syncing servo driver with firmware", end="", flush=True)
        while servo_status == packet.ServoStatus.MISSING_CALIBRATION:
            self.servo_driver.send_calibration()
            needed_to_sync = True
            print(".", end="", flush=True)
            await asyncio.sleep(1)
            servo_status = self._comm.telemetry.servo_status
        print()
        return needed_to_sync

    async def launch_control_loop(
        self, mode: Mode, callback: Callable[[Self], None], fps: int = 20
    ):
        self.servo_driver.send_calibration()
        await asyncio.sleep(0.3)
        frame_interval = 1.0 / fps
        prev_time = time.time()
        while True:
            if await self.sync_with_firmware():
                prev_time = time.time()

            current_time = time.time()
            dt = current_time - prev_time

            telemetry = self._comm.telemetry
            send_to_robot = callback(telemetry)

            if send_to_robot and mode == RobotController.Mode.Normal:
                foot_positions = self.robot_gait.compute_foot_positions(dt)
                joint_angles = self.robot_ik.find_angles(foot_positions)
                self.servo_driver.set_joint_angles(
                    joint_angles.flatten(), self.robot_specs.joint_ranges
                )

            elapsed_frame_time = time.time() - current_time
            await asyncio.to_thread(
                lambda: time.sleep(max(0, frame_interval - elapsed_frame_time))
            )
            prev_time = current_time

    async def launch(self, mode: Mode, callback: Callable[[Self], None], fps: int = 30):
        await self._comm.connect()
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.launch_control_loop(mode, callback, fps))
            tg.create_task(self._comm.listen_to_sensor_readings())

    async def read_sensors(self):
        return await self._comm.read_sensors()

    @classmethod
    async def create(cls):
        comm = await Comm.create()
        return cls(comm)


async def main():
    controller = await RobotController.create()
    await controller.launch()


if __name__ == "__main__":
    asyncio.run(main())
