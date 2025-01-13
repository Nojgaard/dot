import dm_control.viewer as viewer
import numpy as np
from sb3_contrib import ARS
from stable_baselines3 import PPO

from dot.sim.biped import Biped
from dot.sim.environments import modulate_gait_env
from dot.sim.quadruped import Quadruped
from dot.view.control_gui import ControlGui


def main():
    env = modulate_gait_env(model=Quadruped())
    model_ik = env.task.model_ik
    model_gait = env.task.model_gait

    agent = ARS.load("agent.zip")
    #agent = PPO.load("agent.zip")
    normalizer = agent.get_vec_normalize_env()

    env.reset()

    action_spec = env.action_spec()
    default_action = np.zeros((action_spec.shape))

    # print(action_spec)
    gui = ControlGui(model_ik, model_gait)
    def get_action(time_step):
        env.task.enable_input_controller = gui.enable_controller
        gui.update_model(model_ik, model_gait)
        obs = time_step.observation
        print(obs.shape)
        if normalizer is not None:
            obs = normalizer.normalize_obs(time_step.observation)
        action, _ = agent.predict(obs)
        #print(obs)
        print(action)
        print("reward", time_step.reward)
        return action

    gui.launch()
    viewer.launch(env, get_action)


if __name__ == "__main__":
    main()
