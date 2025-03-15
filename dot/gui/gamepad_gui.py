import dearpygui.dearpygui as dpg

from dot.control.gait import Gait
from dot.control.gamepad import Gamepad
from dot.control.inverse_kinematics import RobotIK


class GamepadGui:
    def __init__(self, robot_ik: RobotIK, robot_gait: Gait):
        self._gamepad = Gamepad()
        self._robot_ik = robot_ik
        self._robot_gait = robot_gait
        self._use_gamepad = dpg.add_checkbox(label="Use Gamepad", default_value=False)

    def use_gamepad(self):
        return dpg.get_value(self._use_gamepad)
    
    def update(self):
        if not self.use_gamepad():
            return
        
        self._gamepad.update_robot_inputs(self._robot_ik, self._robot_gait)
