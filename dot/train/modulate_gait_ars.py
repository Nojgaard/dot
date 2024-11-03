import numpy as np
from dm_control import composer

from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.sim.quadruped import Quadruped

# from ray.rllib.env import DMCEnv
# from ray.rllib.env import DMEnv
from sb3_contrib import ARS
import gymnasium as gym

from sb3_contrib.common.vec_env import AsyncEval

from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

from dot.train.dm_control_wrapper import DMCEnv


def create_env():
    model = Quadruped()
    model_ik = QuadropedIK(
        model.body_length,
        model.body_width,
        model.max_height * 0.7,
        model.hip_offset,
        model.shoulder_length,
        model.wrist_length,
    )
    model_gait = Gait(model_ik.foot_points, step_length=0.05)

    task = ModulateGaitTask(model, model_ik, model_gait)
    env = composer.Environment(
        task,
        random_state=np.random.RandomState(42),
        strip_singleton_obs_buffer_dim=True,
        time_limit=10.0,
    )
    #VecNormalize
    vec_env = make_vec_env(lambda: DMCEnv(env))
    vec_env = VecNormalize(vec_env, norm_reward=False)
    return vec_env


def main():
    model = ARS(
        "MlpPolicy",
        create_env(),
        verbose=1,
        n_delta=16,
        n_top=16,
        delta_std=0.05,
        learning_rate=0.03,
        alive_bonus_offset=0.0,
    )
    n_envs = 8
    async_eval = AsyncEval(
        [lambda: create_env() for _ in range(n_envs)], model.policy
    )
    obs_norm = model.get_vec_normalize_env()
    print("BEFORE LEARN", obs_norm.ret_rms.mean, obs_norm.ret_rms.var)
    # async_eval=None
    #model.learn(100000, async_eval=async_eval, log_interval=5)
    model.learn(100000, async_eval=async_eval, log_interval=5)
    model.save("model.zip")
    obs_norm = model.get_vec_normalize_env()
    print("AFTER LEARN", obs_norm.obs_rms.mean, obs_norm.obs_rms.var, obs_norm.obs_rms.count)
    obs_norm.save("vecnorm.pickle")


if __name__ == "__main__":
    main()
