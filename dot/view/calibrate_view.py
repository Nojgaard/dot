from dot.real.robot_controller import RobotController
from dot.view.calibrate_gui import CalibrateGui
import asyncio


async def main():
    gui = CalibrateGui()
    controller = await RobotController.create()
    gui_task = asyncio.create_task(asyncio.to_thread(gui.launch))

    await asyncio.sleep(0.5)
    controller_task = asyncio.create_task(
        controller.launch(RobotController.Mode.Calibrate, callback=gui.update_servos, fps=10)
    )
    done, pending = await asyncio.wait([gui_task, controller_task], return_when=asyncio.FIRST_COMPLETED)

    for task in done:
        print(f"{task.result()}")
    
    for task in pending:
        task.cancel()
    print("Finished")


asyncio.run(main(), debug=True)
