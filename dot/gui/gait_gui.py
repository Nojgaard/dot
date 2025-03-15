from dot.control.gait import Gait
import dearpygui.dearpygui as dpg
import numpy as np


class GaitGui:
    def __init__(self, gait: Gait):
        self._gait = gait
        self.setup_gui(gait)

    def update(self):
        gait = self._gait
        (
            gait.target_speed,
            gait.lateral_rotation_angle,
            gait.yaw_rate,
            gait.clearance_height,
            gait.penetration_depth,
            gait.swing_time,
        ) = dpg.get_values(
            [
                self._velocity,
                self._lateral_angle,
                self._yaw_rate,
                self._clearance,
                self._penetration,
                self._swing_time,
            ]
        )

    def setup_gui(self, gait: Gait):
        with dpg.tree_node(label="Gait", default_open=True):
            self._velocity = dpg.add_slider_float(
                label="Velocity",
                default_value=gait.target_speed,
                min_value=-1,
                max_value=1,
            )

            self._lateral_angle = dpg.add_slider_float(
                label="Lateral Angle",
                default_value=gait.lateral_rotation_angle,
                min_value=-np.pi / 2,
                max_value=np.pi / 2,
            )

            self._yaw_rate = dpg.add_slider_float(
                label="Yaw Rate",
                default_value=gait.yaw_rate,
                min_value=-1,
                max_value=1,
            )

            dpg.add_text("Parameters")
            self._clearance = dpg.add_slider_float(
                label="Clearence Height",
                default_value=gait.clearance_height,
                min_value=0,
                max_value=0.1,
            )

            self._penetration = dpg.add_slider_float(
                label="Penetration Depth",
                default_value=gait.penetration_depth,
                min_value=0,
                max_value=0.05,
            )

            self._swing_time = dpg.add_slider_float(
                label="Swing time",
                default_value=gait.swing_time,
                min_value=0.1,
                max_value=0.5,
            )


if __name__ == "__main__":
    robot_ik = Gait(np.zeros(12).reshape(4, 3))
    dpg.create_context()
    dpg.create_viewport(title="Control Inputs", width=1200, height=600)
    with dpg.window(tag="Primary Window"):
        gui = GaitGui(robot_ik)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)
    gui.update()
    dpg.start_dearpygui()
    dpg.destroy_context()
