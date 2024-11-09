import numpy as np
from dm_control import composer
import dm_env
from dm_env import specs

from dot.control.gait import Gait
from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.sim.quadruped import Quadruped


def _flatten_spec(spec):
    def extract_min_max(s):
        assert s.dtype == np.float64 or s.dtype == np.float32
        dim = np.int_(np.prod(s.shape))
        if type(s) == specs.Array:
            bound = np.inf * np.ones(dim, dtype=np.float32)
            return -bound, bound
        elif type(s) == specs.BoundedArray:
            zeros = np.zeros(dim, dtype=np.float32)
            return s.minimum + zeros, s.maximum + zeros

    mins, maxs = [], []
    for s in spec:
        mn, mx = extract_min_max(s)
        mins.append(mn)
        maxs.append(mx)
    low = np.concatenate(mins, axis=0)
    high = np.concatenate(maxs, axis=0)
    assert low.shape == high.shape

    return specs.BoundedArray(
        shape=low.shape, dtype=np.float32, minimum=low, maximum=high
    )


def _flatten_obs(obs):
    obs_pieces = []
    for v in obs.values():
        flat = np.array([v]) if np.isscalar(v) else v.ravel()
        obs_pieces.append(flat)
    return np.concatenate(obs_pieces, axis=0)


class LearningEnvironment(dm_env.Environment):
    """Environment that wraps another environment.

    This exposes the wrapped environment with the `.environment` property and also
    defines `__getattr__` so that attributes are invisibly forwarded to the
    wrapped environment (and hence enabling duck-typing).
    """

    _env: dm_env.Environment

    def __init__(self, env: dm_env.Environment):
        self._env = env
        self._observation_spec = _flatten_spec(env.observation_spec().values())

    def __getattr__(self, attr: str):
        # Delegates attribute calls to the wrapped environment.
        return getattr(self._env, attr)

    # Getting/setting of state is necessary so that getattr doesn't delegate them
    # to the wrapped environment. This makes sure pickling a wrapped environment
    # works as expected.

    def __getstate__(self):
        return self.__dict__

    def __setstate__(self, state):
        self.__dict__.update(state)

    @property
    def environment(self) -> dm_env.Environment:
        return self._env

    def step(self, action) -> dm_env.TimeStep:
        timestep = self._env.step(action)
        return timestep._replace(
            observation=_flatten_obs(timestep.observation))

    def reset(self) -> dm_env.TimeStep:
        timestep = self._env.reset()
        return timestep._replace(
            observation=_flatten_obs(timestep.observation))

    def action_spec(self):
        return self._env.action_spec()

    def discount_spec(self):
        return self._env.discount_spec()

    def observation_spec(self):
        return self._observation_spec

    def reward_spec(self):
        return self._env.reward_spec()

    def close(self):
        return self._env.close()


def modulate_gait_env():
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
    env = composer.Environment(
        task,
        random_state=np.random.RandomState(42),
        strip_singleton_obs_buffer_dim=True,
    )

    return LearningEnvironment(env)
