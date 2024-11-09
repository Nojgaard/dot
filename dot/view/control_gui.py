from functools import partial
from multiprocessing import Process, Value
import dearpygui.dearpygui as dpg
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK


class ControlInput:
    def __init__(self, name: str, init_value: float, bounds: tuple[float, float]):
        self.name = name
        self.value = Value("d", init_value)
        self.bounds = bounds


def _update_value(sender, data, user_data):
    user_data.value = data


def _add_slider(name, ctrl_value, value_range):
    dpg.add_text(name)
    dpg.add_slider_float(
        label="float",
        user_data=ctrl_value,
        callback=_update_value,
        default_value=ctrl_value.value,
        max_value=value_range[1],
        min_value=value_range[0],
    )


def _open_gui_window(control_inputs: dict[str, list[ControlInput]]):
    dpg.create_context()
    dpg.create_viewport(title="Control Inputs", width=1200, height=600)

    with dpg.window(tag="Primary Window"):
        for title, inputs in control_inputs.items():
            dpg.add_text(title)
            for input in inputs:
                _add_slider(input.name, input.value, input.bounds)

    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)
    dpg.start_dearpygui()
    dpg.destroy_context()


class ControlGui:
    def __init__(self):
        self._control_inputs = {
            "Body Translation": [
                ControlInput("x", 0, (-0.4, 0.4)),
                ControlInput("y", 0, (-0.4, 0.4)),
                ControlInput("z", 0, (-0.4, 0.4)),
            ],
            "Body Rotation": [
                ControlInput("roll", 0, (-1, 1)),
                ControlInput("pitch", 0, (-1, 1)),
                ControlInput("yaw", 0, (-1, 1)),
            ],
            "Gait": [
                ControlInput("step length", 0, (-0.1, 0.1)),
                ControlInput("lateral angle", 0, (-np.pi / 2, np.pi / 2)),
                ControlInput("yaw rate", 0, (-1, 1)),
                ControlInput("velocity", 0.1, (-0.3, 0.3)),
                ControlInput("clearance height", 0.03, (0, 0.1)),
                ControlInput("penetration depth", 0.005, (0, 0.05)),
            ],
        }

        self._process = Process(
            target=_open_gui_window,
            kwargs={"control_inputs": self._control_inputs},
        )

    @property
    def ctrl_translation(self) -> NDArray[np.float64]:
        return np.array(
            [i.value.value for i in self._control_inputs["Body Translation"]],
            dtype=np.float64,
        )

    @property
    def crtl_rotation(self) -> Rotation:
        euler_angles = [i.value.value for i in self._control_inputs["Body Rotation"]]
        return Rotation.from_euler("XYZ", euler_angles, degrees=False)
    
    def _find_input(self, name: str):
        for inputs in self._control_inputs.values():
            for input in inputs:
                if name == input.name:
                    return input
        raise ValueError(f"Cant find name {name}")

    @property
    def step_length(self):
        return self._find_input("step length").value.value

    @property
    def lateral_angle(self):
        return self._find_input("lateral angle").value.value

    @property
    def yaw_rate(self):
        return self._find_input("yaw rate").value.value

    @property
    def penetration_depth(self):
        return self._find_input("penetration depth").value.value

    @property
    def clearance_height(self):
        return self._find_input("clearance height").value.value
    
    @property
    def velocity(self):
        return self._find_input("velocity").value.value

    def launch(self):
        self._process.start()

    def update_model(self, model_ik: QuadropedIK, model_gait: Gait):
        model_ik.translation = self.ctrl_translation
        model_ik.rotation = self.crtl_rotation

        model_gait.step_length = self.step_length
        model_gait.lateral_rotation_angle = self.lateral_angle
        model_gait.yaw_rate = self.yaw_rate
        model_gait.target_speed = self.velocity
        model_gait.clearance_height = self.clearance_height
        model_gait.penetration_depth = self.penetration_depth
