from enum import Enum
import pickle
import numpy as np
from dm_control import composer
import dm_env
from dm_env import specs

from dot.control.gait import Gait
from dot.control.inverse_kinematics import RobotIK
from dot.sim.bumpy_arena import BumpyArena
from dot.sim.gait_input_controller import GaitInputController
from dot.sim.modulate_gait_task import ModulateGaitTask
from dot.sim.quadruped import Quadruped
from dot.sim.robot import Robot
from dm_control.locomotion.arenas import floors


def _scale_action(action: np.ndarray, spec: specs.Array):
    """Converts a single canonical action back to the given action spec."""
    # Get scale and offset of output action spec.
    scale = spec.maximum - spec.minimum
    offset = spec.minimum

    # Maybe clip the action.
    action = np.clip(action, -1.0, 1.0)

    # Map action to [0, 1].
    action = 0.5 * (action + 1.0)

    # Map action to [spec.minimum, spec.maximum].
    action *= scale
    action += offset

    return action


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

        x = np.zeros(env.action_spec().shape)
        self._norm_action_spec = specs.BoundedArray(
            x.shape, np.float32, x - 1, x + 1, "norm action"
        )

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
        scaled_action = _scale_action(action, self._env.action_spec())
        timestep = self._env.step(scaled_action)
        return timestep._replace(observation=_flatten_obs(timestep.observation))

    def reset(self) -> dm_env.TimeStep:
        timestep = self._env.reset()
        return timestep._replace(observation=_flatten_obs(timestep.observation))

    def action_spec(self):
        return self._norm_action_spec
        # return self._env.action_spec()

    def discount_spec(self):
        return self._env.discount_spec()

    def observation_spec(self):
        return self._observation_spec

    def reward_spec(self):
        return self._env.reward_spec()

    def close(self):
        return self._env.close()


class ArenaType(Enum):
    Flat = floors.Floor(reflectance=0.0)
    Bumpy = BumpyArena()


def modulate_gait_env(
    model: Robot = Quadruped(),
    time_limit=float("inf"),
    time_per_mode=5,
    arena_type: ArenaType = ArenaType.Flat,
):
    model_ik = RobotIK(
        model.body_length,
        model.body_width,
        model.max_height * 0.7,
        model.hip_offset,
        model.arm_length,
        model.wrist_length,
        translation=np.array([-0.04, 0, 0]),
    )
    model_gait = Gait(model_ik.foot_points)
    input_controller = GaitInputController(model_gait, time_per_mode=time_per_mode)
    task = ModulateGaitTask(model, model_ik, model_gait, input_controller, arena_type.value)
    env = composer.Environment(
        task,
        time_limit,
        random_state=np.random.RandomState(42),
        strip_singleton_obs_buffer_dim=True,
    )
    # return env
    return LearningEnvironment(env)
