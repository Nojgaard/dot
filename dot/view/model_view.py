from dot.sim.quadruped import Quadruped
import dm_control.mjcf as mjcf
from dm_control.composer.arena import Arena
import mujoco.viewer as mjv


spot = Quadruped()
arena = Arena()
arena.add_free_entity(spot)
arena.mjcf_model.option.timestep = "0.005"
arena.mjcf_model.option.gravity = "0 0 0"
# arena.mjcf_model.worldbody.add('light', pos=(0, 0, 4))
physics = mjcf.Physics.from_mjcf_model(arena.mjcf_model)
mjv.launch(physics.model.ptr, physics.data.ptr)