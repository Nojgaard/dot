import asyncio

import dearpygui.dearpygui as dpg
import numpy as np

from dot.real.packet import Telemetry
from dot.real.robot_controller import RobotController


class CalibrateGui:
    def __init__(self, controller: RobotController):
        self._controller = controller

    def launch(self):
        dpg.create_context()
        dpg.create_viewport(title="Control Inputs", width=1200, height=600)

        with dpg.window(tag="Primary Window"):
            dpg.add_text("Angle")
            self.servo_angle_slider = dpg.add_slider_int(min_value=0, max_value=180)

            dpg.add_text("Min MS PWM")
            self.servo_min_us_slider = dpg.add_slider_int(
                default_value=500, min_value=300, max_value=700
            )

            dpg.add_text("Max MS PWM")
            self.servo_max_us_slider = dpg.add_slider_int(
                default_value=2650, min_value=2400, max_value=2800
            )

            dpg.add_text("Orientation")
            self.servo_orientation_combo = dpg.add_combo(("Left", "Right"), default_value="Left")

            dpg.add_text("Joint Type")
            self.joint_type_combo = dpg.add_combo(
                ("Wrist", "Arm", "Shoulder"), default_value="Wrist"
            )

            dpg.add_button(
                label="Set Nominal Joint Angle", callback=self.set_nominal_joint_angle
            )

            dpg.add_text("Connect")
            self.connect_checkbox = dpg.add_checkbox(default_value=False)

            self._voltage_text = dpg.add_text("Voltage: None")
            self._current_text = dpg.add_text("Current: None")

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def set_nominal_joint_angle(self):
        joint_type = dpg.get_value(self.joint_type_combo)
        type2idx = {"Wrist": 2, "Arm": 1, "Shoulder": 0}

        joint_ranges = self._controller.robot_specs.joint_ranges[
            type2idx[joint_type]
        ].reshape((1, 2))
        angles = np.array([0])
        orientation: str = dpg.get_value(self.servo_orientation_combo)
        invert = [False] * 12 if orientation == "Left" else [True] * 12
        self._controller.servo_driver.calibration.invert_servo_angle = invert
        servo_angle = self._controller.servo_driver.joint_to_servo_angle(
            angles, joint_ranges
        )[0]

        dpg.set_value(self.servo_angle_slider, int(np.round(servo_angle)))

    def update_servos(self, sensor_readings: Telemetry):
        dpg.set_value(
            self._voltage_text, f"Voltage: {sensor_readings.battery_voltage:.2f}"
        )
        dpg.set_value(
            self._current_text, f"Current: {sensor_readings.battery_current:.2f}"
        )

        send_commands = dpg.get_value(self.connect_checkbox)
        if not send_commands:
            return

        angle: float = dpg.get_value(self.servo_angle_slider)
        min_us: float = dpg.get_value(self.servo_min_us_slider)
        max_us: float = dpg.get_value(self.servo_max_us_slider)

        print(f"Updating Servos: angle={angle}, ({min_us}, {max_us})")
        driver = self._controller.servo_driver
        driver.calibration.bounds_pwm_ms = [(min_us, max_us) for _ in range(driver.calibration.num_servos)]
        self._controller.servo_driver.set_servo_angles([angle] * 12)
