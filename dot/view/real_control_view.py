from dot.control.gamepad import Gamepad
from dot.real.comm import Telemetry
from dot.real.robot_controller import RobotController
import dearpygui.dearpygui as dpg
import numpy as np
import asyncio
from scipy.spatial.transform import Rotation


class RealControlGui:
    def __init__(self, controller: RobotController):
        self._controller = controller
        self._gamepad = Gamepad()

    @property
    def joint_angles(self):
        return self._physics.bind(self._joints).qpos.copy()

    def launch(self):
        dpg.create_context()
        dpg.create_viewport(title="Control Inputs", width=1200, height=600)

        with dpg.window(tag="Primary Window"):
            dpg.add_text("Body Translation")
            self._trans_sliders = [
                dpg.add_slider_float(
                    label="x", default_value=-0.015, min_value=-0.1, max_value=0.1
                ),
                dpg.add_slider_float(
                    label="y", default_value=0, min_value=-0.1, max_value=0.1
                ),
                dpg.add_slider_float(
                    label="z", default_value=0.02, min_value=-0.1, max_value=0.1
                ),
            ]

            dpg.add_text("Body Rotation")
            self._rot_sliders = [
                dpg.add_slider_float(
                    label="roll", default_value=0, min_value=-1, max_value=1
                ),
                dpg.add_slider_float(
                    label="pitch", default_value=-0.08, min_value=-1, max_value=1
                ),
                dpg.add_slider_float(
                    label="yaw", default_value=0, min_value=-1, max_value=1
                ),
            ]

            dpg.add_text("Gait")
            self._speed_crtl_sliders = [
                dpg.add_slider_float(
                    label="Velocity", default_value=0, min_value=-1, max_value=1
                ),
                dpg.add_slider_float(
                    label="Lateral Angle",
                    default_value=0,
                    min_value=-np.pi / 2,
                    max_value=np.pi / 2,
                ),
                dpg.add_slider_float(
                    label="Yaw Rate", default_value=0, min_value=-1, max_value=1
                ),
            ]

            self._clearence_sliders = [
                dpg.add_slider_float(
                    label="Clearence Height",
                    default_value=0.03,
                    min_value=0,
                    max_value=0.1,
                ),
                dpg.add_slider_float(
                    label="Penetration Depth",
                    default_value=0.0025,
                    min_value=0,
                    max_value=0.05,
                ),
            ]

            self.connect_checkbox = dpg.add_checkbox(label="Connect to robot", default_value=False)

            self.gamepad_checkbox = dpg.add_checkbox(
                label="Use Gamepad", default_value=False
            )

            self._voltage_text = dpg.add_text("Voltage: None")
            self._current_text = dpg.add_text("Current: None")

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def update_controller(self, sensor_readings: Telemetry):
        dpg.set_value(
            self._voltage_text, f"Voltage: {sensor_readings.battery_voltage:.2f}"
        )
        dpg.set_value(
            self._current_text, f"Current: {sensor_readings.battery_current:.2f}"
        )

        controller = self._controller
        if dpg.get_value(self.gamepad_checkbox):
            self._gamepad.update_robot_inputs(
                controller.robot_ik, controller.robot_gait
            )
        else:
            controller.robot_ik.translation = np.array(
                dpg.get_values(self._trans_sliders)
            )
            euler_angles = np.array(dpg.get_values(self._rot_sliders))
            controller.robot_ik.rotation = Rotation.from_euler(
                "XYZ", euler_angles, degrees=False
            )

            velocity, lateral_angle, yaw_rate = dpg.get_values(self._speed_crtl_sliders)
            controller.robot_gait.target_speed = velocity
            controller.robot_gait.lateral_rotation_angle = lateral_angle
            controller.robot_gait.yaw_rate = yaw_rate

        send_commands = dpg.get_value(self.connect_checkbox)
        return send_commands


async def main():
    controller = await RobotController.create()
    controller.servo_driver.load_calibration("data/calibration.json")
    controller.robot_gait.swing_time = 0.25
    gui = RealControlGui(controller)
    gui_task = asyncio.create_task(asyncio.to_thread(gui.launch))

    await asyncio.sleep(0.5)
    controller_task = asyncio.create_task(
        controller.launch(
            RobotController.Mode.Normal, callback=gui.update_controller, fps=40
        )
    )

    done, pending = await asyncio.wait(
        [gui_task, controller_task], return_when=asyncio.FIRST_COMPLETED
    )

    for task in done:
        if task.result() is not None:
            print(f"{task.result()}")

    for task in pending:
        task.cancel()


asyncio.run(main())
