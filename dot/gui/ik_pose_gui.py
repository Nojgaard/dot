from dot.control.inverse_kinematics import RobotIK
import dearpygui.dearpygui as dpg
import numpy as np
from scipy.spatial.transform import Rotation


class IkPoseGui:
    def __init__(self, robot_ik: RobotIK):
        self._robot_ik = robot_ik
        self.setup_gui()

    def translation(self):
        return np.array(dpg.get_values(self._sliders_translation))

    def rotation(self):
        euler_angles = np.array(dpg.get_values(self._sliders_rotation))
        return Rotation.from_euler("XYZ", euler_angles, degrees=False)
    
    def update(self):
        self._robot_ik.translation = self.translation()
        self._robot_ik.rotation = self.rotation()

    def setup_gui(self):
        translation = self._robot_ik.translation
        rotation = self._robot_ik.rotation.as_euler("XYZ")
        with dpg.tree_node(label="Body Translation", default_open=True):
            with dpg.group(horizontal=True):
                self._sliders_translation = [
                    dpg.add_slider_float(
                        label=name,
                        default_value=translation[i],
                        min_value=-0.1,
                        max_value=0.1,
                        width=150,
                    )
                    for i, name in enumerate(["x", "y", "z"])
                ]

        with dpg.tree_node(label="Body Rotation", default_open=True):
            with dpg.group(horizontal=True):
                self._sliders_rotation = [
                    dpg.add_slider_float(
                        label=name,
                        default_value=rotation[i],
                        min_value=-0.4,
                        max_value=0.4,
                        width=150,
                    )
                    for i, name in enumerate(["x", "y", "z"])
                ]


def main():
    robot_ik = RobotIK(0, 0, 0, [0, 0], 0, 0)
    dpg.create_context()
    dpg.create_viewport(title="Control Inputs", width=1200, height=600)
    with dpg.window(tag="Primary Window"):
        gui = IkPoseGui(robot_ik)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)
    dpg.start_dearpygui()
    dpg.destroy_context()


if __name__ == "__main__":
    main()
