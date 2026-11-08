import mujoco,numpy as np,json
from pillbot.sim.world import SimWorld
from pathlib import Path
u=mujoco.MjModel.from_xml_path('hardware/urdf/pillbot.urdf'); ud=mujoco.MjData(u)
w=SimWorld(); s=w.model; sd=w.data
rng=np.random.default_rng(4); worst=0.; angle=0.
names=['base','base_plate','drive_plate','shoulder_1','shoulder_2','link_1','elbow_1','elbow_2','link_2','wrist_gear_1']
pairs = [(n,n) for n in names] + [('wrist_roll_follower_so101','gripper_body'),('moving_jaw_so101','moving_jaw')]
for _ in range(50):
 for name in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_roll','gripper']:
  q=rng.uniform(0,1.5) if name == "gripper" else rng.uniform(-1,1)
  ud.qpos[u.jnt_qposadr[u.joint(name).id]]=q; sd.qpos[s.jnt_qposadr[s.joint(name).id]]=q
 mujoco.mj_forward(u,ud); mujoco.mj_forward(s,sd)
 for un, sn in pairs:
  ug=np.flatnonzero((u.geom_type==mujoco.mjtGeom.mjGEOM_MESH)&(u.geom_dataid==u.mesh(un).id))[0]
  sg=np.flatnonzero((s.geom_type==mujoco.mjtGeom.mjGEOM_MESH)&(s.geom_dataid==s.mesh(sn).id))[0]
  worst=max(worst,float(np.linalg.norm(ud.geom_xpos[ug]-sd.geom_xpos[sg])))
  angle=max(angle,float(np.max(abs(ud.geom_xmat[ug]-sd.geom_xmat[sg]))))
assert worst<1e-8 and angle<1e-8,(worst,angle)
r={'poses':50,'meshes':pairs,'max_position_error_m':worst,'max_rotation_matrix_error':angle,'dynamics_steps':0}
Path('outputs/assembly_review_v2/mesh_parity.json').write_text(json.dumps(r,indent=2)); print(r)
