"""Static model checks: no dynamics or unrecorded control episodes."""
import mujoco
import numpy as np

from pillbot.sim.world import SimWorld
from pillbot.utils.config import load_yaml


def test_fixed_bottle_has_no_dofs_but_cap_can_press_and_twist():
    sim = load_yaml('configs/sim.yaml')
    sim['fixture']['fixed_to_ground'] = True
    world = SimWorld(sim_cfg=sim)
    assert world.free_q is None
    assert world.model.body_dofnum[world.bottle_body] == 0
    assert mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_JOINT,'bottle_free') == -1
    assert world.model.joint('cap_press').type[0] == mujoco.mjtJoint.mjJNT_SLIDE
    assert world.model.joint('cap_rot').type[0] == mujoco.mjtJoint.mjJNT_HINGE
    world.reset(start='zone',pick_xy=[.2,.1])
    np.testing.assert_allclose(world.bottle_pos,[.2,.1,0])
    np.testing.assert_allclose(world.data.xpos[world.bottle_body],world.bottle_pos)
    assert world.bottle_upright() == 0
    world.close()


def test_default_bottle_remains_free_for_pickup():
    world = SimWorld()
    assert not world.fixed_bottle
    assert world.model.body_dofnum[world.bottle_body] == 6
    assert world.free_q is not None
    world.close()
