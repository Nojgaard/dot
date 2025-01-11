from functools import partial
from multiprocessing import Process, Value
import dearpygui.dearpygui as dpg
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from dot.control.gait import Gait
from dot.control.gamepad import Gamepad
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


def _open_gui_window(enable_controller, use_gamepad, control_inputs: dict[str, list[ControlInput]]):
    dpg.create_context()
    dpg.create_viewport(title="Control Inputs", width=1200, height=600)

    with dpg.window(tag="Primary Window"):
        dpg.add_checkbox(
            label="Enable Open Loop Controller",
            user_data=enable_controller,
            callback=_update_value,
        )
        dpg.add_checkbox(
            label="Use Gamepad",
            user_data=use_gamepad,
            callback=_update_value,
        )
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
    def __init__(self, model_ik: QuadropedIK, gait: Gait, enable_controller=False):
        self._enable_controller = Value("i", int(enable_controller))
        self._use_gamepad = Value("i", int(False))
        orientation = model_ik.rotation.as_euler("XYZ", degrees=False)
        self._control_inputs = {
            "Body Translation": [
                ControlInput("x", model_ik.translation[0], (-0.4, 0.4)),
                ControlInput("y", model_ik.translation[1], (-0.4, 0.4)),
                ControlInput("z", model_ik.translation[2], (-0.4, 0.4)),
            ],
            "Body Rotation": [
                ControlInput("roll", orientation[0], (-1, 1)),
                ControlInput("pitch", orientation[1], (-1, 1)),
                ControlInput("yaw", orientation[2], (-1, 1)),
            ],
            "Gait": [
                ControlInput("velocity", gait.target_speed, (-1.0, 1.0)),
                ControlInput(
                    "lateral angle",
                    gait.lateral_rotation_angle,
                    (-np.pi / 2, np.pi / 2),
                ),
                ControlInput("yaw rate", gait.yaw_rate, (-1, 1)),
                ControlInput("clearance height", gait.clearance_height, (0, 0.1)),
                ControlInput("penetration depth", gait.penetration_depth, (0, 0.05)),
            ],
        }

        self._process = Process(
            target=_open_gui_window,
            kwargs={
                "enable_controller": self._enable_controller,
                "use_gamepad": self._use_gamepad,
                "control_inputs": self._control_inputs,
            },
        )
        self._gamepad = Gamepad()

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

    @property
    def enable_controller(self):
        return bool(self._enable_controller.value)

    def launch(self):
        self._process.start()

    def update_model(self, model_ik: QuadropedIK, model_gait: Gait):
        if self.enable_controller:
            return
        if self._use_gamepad.value:
            self._gamepad.update_robot_inputs(model_ik, model_gait)
            return

        model_ik.translation = self.ctrl_translation
        model_ik.rotation = self.crtl_rotation

        model_gait.lateral_rotation_angle = self.lateral_angle
        model_gait.yaw_rate = self.yaw_rate
        model_gait.target_speed = self.velocity
        model_gait.clearance_height = self.clearance_height
        model_gait.penetration_depth = self.penetration_depth
