import dm_control.viewer as viewer
import numpy as np
from sb3_contrib import ARS

from dot.sim.environments import modulate_gait_env
from dot.view.control_gui import ControlGui


def main():
    gui = ControlGui()

    env = modulate_gait_env()
    model_ik = env.task.model_ik
    model_gait = env.task.model_gait

    agent = ARS.load("agent.zip")
    normalizer = agent.get_vec_normalize_env()

    env.reset()

    action_spec = env.action_spec()
    default_action = np.zeros((action_spec.shape))
    default_action[:2] += 0.03

    # print(action_spec)
    def get_action(time_step):
        gui.update_model(model_ik, model_gait)
        obs = normalizer.normalize_obs(time_step.observation)
        action, _ = agent.predict(obs)
        return action

    gui.launch()
    viewer.launch(env, get_action)


if __name__ == "__main__":
    main()
