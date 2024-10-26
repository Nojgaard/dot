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

import mujoco.viewer as mv
from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.quadruped import Quadruped
from dot.sim.task import ModulateGaitTask
import mujoco
from multiprocessing import Process
import time
import dearpygui.dearpygui as dpg
import sys
from scipy.spatial.transform import Rotation

from dot.view.control_gui import ControlGui


def main():

    gui = ControlGui()

    spot = Quadruped()
    model_ik = QuadropedIK(
        spot.body_length,
        spot.body_width,
        spot.max_height * 0.7,
        spot.hip_offset,
        spot.shoulder_length,
        spot.wrist_length,
    )
    model_gait = Gait(model_ik.foot_points)
    initial_angles = model_ik.find_angles()
    task = ModulateGaitTask(spot, np.array(initial_angles).flatten())
    env = composer.Environment(task, random_state=np.random.RandomState(42))
    env.reset()

    action_spec = env.action_spec()
    print(action_spec)

    # print(action_spec)
    # Define a uniform random policy.
    def random_policy(time_step):
        gui.update_model(model_ik, model_gait)
        foot_positions = model_gait.compute_foot_positions(task.control_timestep)
        angles = model_ik.find_angles(foot_positions)
        return np.array(angles).flatten()

    gui.launch()
    viewer.launch(env, random_policy)


if __name__ == "__main__":
    main()
