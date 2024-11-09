from sb3_contrib import ARS
from sb3_contrib.common.vec_env import AsyncEval
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from dot.sim.environments import modulate_gait_env
from dot.train.dm2gym import DmToGymEnv


def create_env():
    dm_env = modulate_gait_env(time_limit=10.0, step_length=0.05)
    vec_env = make_vec_env(lambda: DmToGymEnv(dm_env))
    vec_env = VecNormalize(vec_env, norm_reward=False)
    return vec_env


def main():
    print("Creating Agent")
    agent = ARS(
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
    async_eval = AsyncEval([lambda: create_env() for _ in range(n_envs)], agent.policy)

    print("Learning policy")
    agent.learn(100000, async_eval=async_eval, log_interval=1)

    print("Saving agent")
    agent.save("agent.zip", include=["_vec_normalize_env"])


if __name__ == "__main__":
    main()
