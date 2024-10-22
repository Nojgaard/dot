from functools import partial
from multiprocessing import Process, Value
import dearpygui.dearpygui as dpg
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


def _update_value(sender, data, user_data):
    user_data.value = data


def _add_slider(name, ctrl_value, value_range):
    dpg.add_text(name)
    dpg.add_slider_float(
        label="float",
        user_data=ctrl_value,
        callback=_update_value,
        default_value=ctrl_value.value,
        max_value=value_range,
        min_value=-value_range,
    )


def _open_gui_window(
    crtl_trans_x,
    crtl_trans_y,
    crtl_trans_z,
    crtl_rot_x,
    crtl_rot_y,
    crtl_rot_z,
    crtl_step_length,
    ctrl_lateral_angle,
    crtl_yaw_rate,
    crtl_clearance_height,
    crtl_penetration_depth,
):
    dpg.create_context()
    dpg.create_viewport(title="Control Inputs", width=1200, height=600)

    with dpg.window(tag="Primary Window"):
        dpg.add_text("Body Translation")
        _add_slider("x", crtl_trans_x, 0.4)
        _add_slider("y", crtl_trans_y, 0.4)
        _add_slider("z", crtl_trans_z, 0.4)

        dpg.add_text("Body Rotation")
        _add_slider("roll", crtl_rot_x, 1)
        _add_slider("pitch", crtl_rot_y, 1)
        _add_slider("yaw", crtl_rot_z, 1)

        dpg.add_text("Gait")
        _add_slider("Step Length", crtl_step_length, 0.1)
        _add_slider("Lateral Angle", ctrl_lateral_angle, np.pi / 2)
        _add_slider("Yaw Rate", crtl_yaw_rate, 1)
        _add_slider("Clearance Height", crtl_clearance_height, 0.1)
        _add_slider("Penetration Depth", crtl_penetration_depth, 0.05)

    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)
    dpg.start_dearpygui()
    dpg.destroy_context()


class ControlGui:
    def __init__(self):
        self._crtl_trans_x = Value("d", 0.0)
        self._crtl_trans_y = Value("d", 0.0)
        self._crtl_trans_z = Value("d", 0.0)

        self._crtl_rot_x = Value("d", 0.0)
        self._crtl_rot_y = Value("d", 0.0)
        self._crtl_rot_z = Value("d", 0.0)

        self._crtl_step_length = Value("d", 0.0)
        self._ctrl_lateral_angle = Value("d", 0.0)
        self._ctrl_yaw_rate = Value("d", 0.0)
        self._crtl_clearance_height = Value("d", 0.03)
        self._crtl_penetration_depth = Value("d", 0.005)

        self._process = Process(
            target=_open_gui_window,
            args=(
                self._crtl_trans_x,
                self._crtl_trans_y,
                self._crtl_trans_z,
                self._crtl_rot_x,
                self._crtl_rot_y,
                self._crtl_rot_z,
                self._crtl_step_length,
                self._ctrl_lateral_angle,
                self._ctrl_yaw_rate,
                self._crtl_clearance_height,
                self._crtl_penetration_depth,
            ),
        )

    @property
    def ctrl_translation(self) -> NDArray[np.float64]:
        return np.array(
            [
                self._crtl_trans_x.value,
                self._crtl_trans_y.value,
                self._crtl_trans_z.value,
            ],
            dtype=np.float64,
        )

    @property
    def crtl_rotation(self) -> Rotation:
        euler_angles = [
            self._crtl_rot_x.value,
            self._crtl_rot_y.value,
            self._crtl_rot_z.value,
        ]
        return Rotation.from_euler("XYZ", euler_angles, degrees=False)
    
    @property
    def step_length(self):
        return self._crtl_step_length.value
    
    @property
    def lateral_angle(self):
        return self._ctrl_lateral_angle.value
    
    @property
    def yaw_rate(self):
        return self._ctrl_yaw_rate.value
    
    @property
    def penetration_depth(self):
        return self._crtl_penetration_depth.value
    
    @property
    def clearance_height(self):
        return self._crtl_clearance_height.value

    def launch(self):
        self._process.start()
