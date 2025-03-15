import dearpygui.dearpygui as dpg

from dot.real.packet import Telemetry


class CommGui:
    def __init__(self):
        with dpg.tree_node(label="Comms", default_open=True):
            self._send_to_robot = dpg.add_checkbox(label="Send to robot", default_value=False)
            with dpg.tree_node(label="Telemetry", default_open=True):
                self._battery_status = dpg.add_text()
                self._device_status = dpg.add_text()

    def update(self, telemetry: Telemetry):
        dpg.set_value(
            self._battery_status,
            f"Battery: {telemetry.battery_voltage:.2f}V {telemetry.battery_current:.2f}A",
        )
        dpg.set_value(
            self._device_status,
            f"Servo: [{telemetry.servo_status.name}] IMU: [{telemetry.imu_status}]"
        )

    def send_to_robot(self):
        return dpg.get_value(self._send_to_robot)
