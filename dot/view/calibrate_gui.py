import asyncio

import dearpygui.dearpygui as dpg

from dot.real.robot_controller import RobotController


class CalibrateGui:
    def __init__(self):
        pass

    def launch(self):
        dpg.create_context()
        dpg.create_viewport(title="Control Inputs", width=1200, height=600)

        with dpg.window(tag="Primary Window"):
            dpg.add_text("Angle")
            self.servo_angle_slider = dpg.add_slider_int(min_value=0, max_value=180)

            dpg.add_text("Min MS PWM")
            self.servo_min_us_slider = dpg.add_slider_int(
                default_value=500, min_value=400, max_value=700
            )

            dpg.add_text("Max MS PWM")
            self.servo_max_us_slider = dpg.add_slider_int(
                default_value=2500, min_value=2400, max_value=2700
            )

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def update_servos(self, controller: RobotController):
        angle: float = dpg.get_value(self.servo_angle_slider)
        min_us: float = dpg.get_value(self.servo_min_us_slider)
        max_us: float = dpg.get_value(self.servo_max_us_slider)

        print(f"Updating Servos: angle={angle}, ({min_us}, {max_us})")
        controller.servo_driver.update_calibration((min_us, max_us))
        controller.servo_driver.set_angles([angle] * 12)
