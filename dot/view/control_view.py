import dm_control.viewer as viewer
import numpy as np
from dm_control import composer

from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.quadruped import Quadruped
from dot.sim.task import ModulateGaitTask
from dot.view.control_gui import ControlGui


def main():

    gui = ControlGui()

    model = Quadruped()
    model_ik = QuadropedIK(
        model.body_length,
        model.body_width,
        model.max_height * 0.7,
        model.hip_offset,
        model.shoulder_length,
        model.wrist_length,
    )
    model_gait = Gait(model_ik.foot_points)
    task = ModulateGaitTask(model, model_ik, model_gait)
    env = composer.Environment(task, random_state=np.random.RandomState(42))
    env.reset()

    action_spec = env.action_spec()
    print(action_spec)

    # print(action_spec)
    def update_gui(time_step):
        gui.update_model(model_ik, model_gait)
        return np.zeros(action_spec.shape)

    gui.launch()
    viewer.launch(env, update_gui)


if __name__ == "__main__":
    main()
