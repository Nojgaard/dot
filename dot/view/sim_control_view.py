import asyncio
import time
import dm_control.viewer as viewer
import numpy as np
from dm_control import composer

from dot.control.gait import Gait
from dot.control.gamepad import Gamepad
from dot.control.inverse_kinematics import RobotIK
from dot.gui.control_gui import ControlGui
from dot.sim.biped import Biped
from dot.sim.quadruped import Quadruped
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.sim.environments import ArenaType, LearningEnvironment, modulate_gait_env

def create_mujoco_viewer(env: LearningEnvironment, gui: ControlGui):
    action_spec = env.action_spec()
    robot: Quadruped = env.task.model
    robot_ik: RobotIK = env.task.model_ik
    robot_gait: Gait = env.task.model_gait

    task: ModulateGaitTask = env.task
    def update_gui(time_step):
        gui.update(modulate_gait_task=task)
        action = np.zeros(action_spec.shape)
        #print("reward:", time_step.reward)
        # print(time_step.observation)
        # print(f"TIME {model_gait._time}")
        return action
    return lambda: viewer.launch(env, update_gui)

def main():
    robot = Quadruped()
    env = modulate_gait_env(robot, arena_type=ArenaType.Flat)
    model_ik = env.task.model_ik
    model_gait = env.task.model_gait
    env.reset()

    gui = ControlGui(model_ik, model_gait, show_comm=False)
    gui.launch(create_mujoco_viewer(env, gui))

if __name__ == "__main__":
    main()
