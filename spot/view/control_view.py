# import dm_control.mujoco as mujoco
import dm_control.mujoco
import dm_control.viewer as viewer
from dm_control import composer
from dm_control import suite
import dm_control.viewer
import numpy as np
import sys
import os
import dm_control
print(sys.path)

import mujoco.viewer as mv
from spot.control.gait import Gait
from spot.control.inverse_kinematics import QuadropedIK
from spot.sim.spot import Spot
from spot.sim.task import WalkTask
import mujoco
from multiprocessing import Process
import time
import dearpygui.dearpygui as dpg
import sys
from scipy.spatial.transform import Rotation

from spot.view.control_gui import ControlGui


def update_gait(model_gait: Gait, gui: ControlGui):
    model_gait.step_length = gui.step_length
    model_gait.lateral_rotation_angle = gui.lateral_angle
    model_gait.yaw_rate = gui.yaw_rate
    model_gait.clearance_height = gui.clearance_height
    model_gait.penetration_depth = gui.penetration_depth


def main():

    gui = ControlGui()

    spot = Spot()
    model_ik = QuadropedIK(
        spot.body_length,
        spot.body_width,
        spot.max_height * 0.7,
        spot.hip_offset,
        spot.shoulder_length,
        spot.wrist_length,
    )
    model_gait = Gait(model_ik.foot_points)
    initial_angles = model_ik.find_angles(Rotation.identity(), np.zeros((1, 3)))
    task = WalkTask(spot, np.array(initial_angles).flatten())
    env = composer.Environment(task, random_state=np.random.RandomState(42))
    env.reset()

    action_spec = env.action_spec()
    print(action_spec)

    # print(action_spec)
    # Define a uniform random policy.
    def random_policy(time_step):
        update_gait(model_gait, gui)
        foot_positions = model_gait.compute_foot_positions(task.control_timestep)
        angles = model_ik.find_angles(
            gui.crtl_rotation, gui.ctrl_translation, foot_positions
        )
        return np.array(angles).flatten()

    gui.launch()
    viewer.launch(env, random_policy)


if __name__ == "__main__":
    main()
