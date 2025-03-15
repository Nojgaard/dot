from dot.control.gamepad import Gamepad
from dot.gui.control_gui import ControlGui
from dot.real.robot_controller import RobotController
import dearpygui.dearpygui as dpg
import numpy as np
import asyncio
from scipy.spatial.transform import Rotation


async def main():
    controller = await RobotController.create()
    controller.servo_driver.load_calibration("data/calibration.json")
    controller.robot_gait.swing_time = 0.20
    controller.robot_gait.clearance_height = 0.035

    gui = ControlGui(controller.robot_ik, controller.robot_gait, show_open_loop=False)

    gui_task = asyncio.create_task(asyncio.to_thread(gui.launch))
    controller_task = asyncio.create_task(
        controller.launch(RobotController.Mode.Normal, callback=gui.update, fps=20)
    )

    try:
        await asyncio.wait(
            [gui_task, controller_task], return_when=asyncio.FIRST_COMPLETED
        )
    except Exception as e:
        print(e)


asyncio.run(main())
