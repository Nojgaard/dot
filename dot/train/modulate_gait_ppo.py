from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from dot.sim.biped import Biped
from dot.sim.environments import ArenaType, modulate_gait_env
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.sim.quadruped import Quadruped
from dot.train.dm2gym import DmToGymEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.policies import ActorCriticPolicy


def create_dm_env():
    dm_env = modulate_gait_env(
        model=Quadruped(), time_limit=5.0, time_per_mode=5, arena_type=ArenaType.Flat
    )
    dm_env.task.model_gait.target_speed = 0.4
    dm_env.task.enable_input_controller = False
    return DmToGymEnv(dm_env)


def main():
    # vec_env = SubprocVecEnv([create_dm_env for _ in range(8)], start_method="fork")
    # policy = ActorCriticPolicy()
    vec_env = make_vec_env(create_dm_env, n_envs=8, vec_env_cls=SubprocVecEnv)
    # vec_env = make_vec_env(create_dm_env, n_envs=1)
    vec_env = VecNormalize(vec_env, norm_reward=False, clip_reward=30.0)
    model = PPO(
        "MlpPolicy",
        vec_env,
        verbose=1,
        device="auto",
        use_sde=False,
        max_grad_norm=1.0,
        n_steps=8192,
        batch_size=32,
        target_kl=0.01,
        n_epochs=50,
        learning_rate=0.0003,
        ent_coef=0.00,
        vf_coef=0.5,
        policy_kwargs={
            # "squash_output": True,
            "net_arch": dict(pi=[32, 32], vf=[32, 32]),
            "ortho_init": True,
            "use_expln": False,
        },
    )
    # print(model.get_parameters())
    model.learn(total_timesteps=5000000)
    model.save("agent.zip", include=["_vec_normalize_env"])
    # print(model.get_parameters())


if __name__ == "__main__":
    main()
