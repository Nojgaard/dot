import asyncio
from threading import Thread
from typing import Optional
import dearpygui.dearpygui as dpg

from dot.control.gait import Gait
from dot.control.inverse_kinematics import RobotIK
from dot.gui.comm_gui import CommGui
from dot.gui.gait_gui import GaitGui
from dot.gui.gamepad_gui import GamepadGui
from dot.gui.ik_pose_gui import IkPoseGui
from dot.gui.open_loop_gui import OpenLoopGui
from dot.real.packet import Telemetry
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.sim.quadruped import Quadruped


class ControlGui:
    def __init__(
        self, robot_ik: RobotIK, robot_gait: Gait, show_open_loop=True, show_comm=True
    ):
        dpg.create_context()
        dpg.create_viewport(title="Control Gui", width=1200, height=600)
        with dpg.window(tag="Primary Window"):
            self._gui_comm = CommGui() if show_comm else None
            self._gui_ik = IkPoseGui(robot_ik)
            self._gui_gait = GaitGui(robot_gait)
            with dpg.group(horizontal=True):
                self._gui_gamepad = GamepadGui(robot_ik, robot_gait)
                self._gui_open_loop = OpenLoopGui() if show_open_loop else None

    def launch(self, background_thread=None):
        if background_thread is not None:
            Thread(target=background_thread, daemon=True).start()

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def update(
        self,
        telemetry: Optional[Telemetry] = None,
        modulate_gait_task: Optional[ModulateGaitTask] = None,
    ):
        if telemetry is not None and self._gui_comm is not None:
            self._gui_comm.update(telemetry)

        if self._gui_open_loop is not None and modulate_gait_task is not None:
            modulate_gait_task.enable_input_controller = (
                self._gui_open_loop.is_enabled()
            )

        self._gui_ik.update()
        self._gui_gait.update()
        self._gui_gamepad.update()
        return self._gui_comm is not None and self._gui_comm.send_to_robot()


if __name__ == "__main__":
    quad = Quadruped()
    ik = RobotIK.from_quadruped(quad)
    gait = Gait(ik.foot_points)
    gui = ControlGui(ik, gait)
    gui.launch()
