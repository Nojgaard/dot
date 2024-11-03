import dm_control.viewer as viewer
import numpy as np
from dm_control import composer

from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.quadruped import Quadruped
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.train.dm_control_wrapper import DMCEnv
from dot.view.control_gui import ControlGui
from scipy.spatial.transform import Rotation
from sb3_contrib import ARS
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

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
    model_gait = Gait(model_ik.foot_points, step_length=0.035)

    task = ModulateGaitTask(model, model_ik, model_gait)
    env = composer.Environment(
        task, random_state=np.random.RandomState(42), strip_singleton_obs_buffer_dim=True,
        time_limit=25.0
    )

    gym_env = DMCEnv(env)
    ml_model = ARS.load("model.zip")
    obs_norm = VecNormalize.load("vecnorm.pickle", make_vec_env(lambda: gym_env))
    
    env.reset()

    action_spec = env.action_spec()
    default_action = np.zeros((action_spec.shape))
    default_action[:2] += 0.03

    # print(action_spec)
    def get_action(time_step):
        gui.update_model(model_ik, model_gait)
        obs = gym_env._get_obs(time_step)
        obs = obs_norm.normalize_obs(obs)
        norm_action, _ = ml_model.predict(obs)
        #return default_action
        action = gym_env._convert_action(norm_action)
        print(action)
        #print(norm_action)
        return action

    gui.launch()
    viewer.launch(env, get_action)


if __name__ == "__main__":
    main()
