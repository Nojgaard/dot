import time
import dm_control.viewer as viewer
import numpy as np
from dm_control import composer

from dot.control.gait import Gait
from dot.control.gamepad import Gamepad
from dot.control.inverse_kinematics import RobotIK
from dot.sim.biped import Biped
from dot.sim.quadruped import Quadruped
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.view.control_gui import ControlGui
from scipy.spatial.transform import Rotation
from dot.sim.environments import modulate_gait_env


def main():
    robot = Quadruped()
    env = modulate_gait_env(robot)
    task: ModulateGaitTask = env.task
    model_ik = env.task.model_ik
    model_gait = env.task.model_gait
    env.reset()

    action_spec = env.action_spec()
    print(action_spec)
    gui = ControlGui(model_ik, model_gait)
    def update_gui(time_step):
        task.enable_input_controller = gui.enable_controller
        gui.update_model(model_ik, model_gait)
        action = np.zeros(action_spec.shape)
        action[0] = gui.penetration_depth
        action[1] = gui.clearance_height
        print("reward:", time_step.reward)
        # print(time_step.observation)
        # print(f"TIME {model_gait._time}")
        return action

    gui.launch()
    viewer.launch(env, update_gui)

if __name__ == "__main__":
    main()
