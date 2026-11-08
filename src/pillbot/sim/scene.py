"""Build the MuJoCo model (MJCF XML) of the arm, the clamped bottle and the cap from configs/sim.yaml.

Generated instead of hand-written so arm measurements and bottle profiles (configs/bottles.yaml)
change the model without editing XML. Units: meters, kg, radians.

    python -m pillbot.sim.scene > /tmp/pillbot.xml     # inspect the generated model

Model, top to bottom:
  table (z = 0) -> riser -> base [shoulder_pan] -> upper_arm [shoulder_lift] -> forearm [elbow_flex]
  -> hand [wrist_roll] = SO-101 gripper body + fixed jaw -> moving_jaw [gripper: hinge]
     + wrist camera, + 5x5 SensX pad on the moving jaw. Your STLs (hardware/stl) are the visuals;
     collisions use boxes fitted to the real jaw faces (meshes would collide as convex blobs).
  bottle (fixed in the clip) -> cap_thread [cap_rot: hinge, cap_lift: slide] -> cap [cap_press: spring]
"""
import math

from pillbot.utils.config import ROOT, get_bottle, load_yaml

TAXELS = 5
LOOK_CM = (10, 8, 7, 6, 5)     # points this many cm in front of the wrist camera (aiming it = IK on one)
PAD_THICK = 0.0005          # tactile/TPU pad half-thickness (1 mm pads)
STL = ROOT / "hardware/stl"

# SO-101 gripper geometry (hardware/stl/gripper_so101, frames from TheRobotStudio's so101 URDF), meters,
# in the HAND frame: origin on the wrist_roll axis, +z out along the fingers.
JAW_HINGE = (0.0202, -0.0188, 0.0234)       # moving-jaw hinge; axis = hand +y, + opens
FIXED_FACE = ((-0.0133, 0.050), (-0.0079, 0.100))     # fixed jaw inner face (x, z) at two heights
MOVING_FACE = ((0.0119, 0.055), (0.0079, 0.100))      # moving jaw inner face at angle 0 (x, z)


def _face(p0, p1):
    """Midpoint, tilt about +y (deg) and length of a jaw face given two (x, z) points."""
    (x0, z0), (x1, z1) = p0, p1
    return ((x0 + x1) / 2, (z0 + z1) / 2), math.degrees(math.atan2(x1 - x0, z1 - z0)), math.hypot(x1 - x0, z1 - z0)


def fixed_face_x(z: float) -> float:
    (x0, z0), (x1, z1) = FIXED_FACE
    return x0 + (x1 - x0) * (z - z0) / (z1 - z0)


def _pads(pitch: float) -> str:
    """5x5 SensX taxels in a frame whose x = 0 plane is the jaw face (pads stick out toward -x)."""
    out = []
    for r in range(TAXELS):            # row 0 = toward the fingertip, col 0 = -y side
        for c in range(TAXELS):
            y, z = (c - 2) * pitch, (2 - r) * pitch
            out.append(
                f'<geom name="taxel_{r}_{c}" class="pad" type="box" size="{PAD_THICK} {pitch/2 - 0.0002} {pitch/2 - 0.0002}" '
                f'pos="{-PAD_THICK} {y} {z}" rgba="0.2 0.5 0.9 1"/>'
                f'<site name="taxel_{r}_{c}" type="box" size="{PAD_THICK * 3} {pitch/2} {pitch/2}" pos="{-PAD_THICK} {y} {z}" rgba="0 0 0 0"/>')
    return "".join(out)


def _pusher_xml(a: dict, grasp) -> str:
    """Optional printed block between the jaws whose bottom face meets the cap TOP when the cap sits in
    the jaws: the child-proof push then goes straight into the cap instead of through jaw friction."""
    if not a.get("palm_pusher"):
        return ""
    bottom = grasp[2] - a["cap_height"] / 2
    return (f'<geom name="pusher" class="solid" type="box" size="0.008 0.007 0.0125" '
            f'pos="{grasp[0]} 0 {bottom - 0.0125}" rgba="0.3 0.3 0.35 1" mass="0.005"/>')


def _mesh(name, file, **kw) -> str:
    extra = " ".join(f'{k}="{v}"' for k, v in kw.items())
    return f'<mesh name="{name}" file="{STL / file}" scale="0.001 0.001 0.001" {extra}/>'


MESHES = "".join([
    _mesh("base", "arm/base.stl"), _mesh("base_plate", "arm/base_plate.stl"),
    _mesh("link_1", "arm/link_1.stl"), _mesh("elbow_1", "arm/elbow_1.stl"), _mesh("elbow_2", "arm/elbow_2.stl"),
    _mesh("link_2", "arm/link_2.stl"), _mesh("wrist_gear_1", "arm/wrist_gear_1.stl"),
    _mesh("gripper_body", "gripper_so101/wrist_roll_follower_so101.stl"),
    _mesh("moving_jaw", "gripper_so101/moving_jaw_so101.stl"),
])


def _vis(mesh, pos_mm, rgba="0.95 0.55 0.15 1", **kw) -> str:
    """A mesh as a visual-only geom; pos_mm shifts it from its STL (assembly) frame into the body frame."""
    extra = " ".join(f'{k}="{v}"' for k, v in kw.items())
    x, y, z = (v / 1000 for v in pos_mm)
    return f'<geom type="mesh" mesh="{mesh}" pos="{x} {y} {z}" rgba="{rgba}" contype="0" conaffinity="0" mass="0" {extra}/>'


def grasp_point(a: dict, cap_r: float) -> tuple[float, float, float]:
    """Hand-frame point where the cap's centre sits when it rests against the fixed jaw."""
    z = a["grasp_depth"]
    return (fixed_face_x(z) + 2 * PAD_THICK + cap_r, 0.0, z)


def _arm_xml(a: dict, cam: dict, cap_r: float) -> str:
    lim = a["limits_deg"]
    L1, L2 = a["upper_arm"], a["forearm"]
    pitch = a.get("taxel_pitch", 0.004)
    m = a["link_mass"]
    meshes = a.get("meshes", True)
    grasp = grasp_point(a, cap_r)
    (fx_, fz_), f_tilt, f_len = _face(*FIXED_FACE)
    (mx_, mz_), m_tilt, m_len = _face(*MOVING_FACE)
    hx, hy, hz = JAW_HINGE
    jr = a["jaw_range_deg"]
    # STL assembly frame -> body frames: link_1 starts at z=166.3 mm, link_2 at 345.6 mm (hardware/stl/README.md)
    upper_vis = (_vis("link_1", (0.8, -6.05, -166.3)) + _vis("elbow_1", (0.8, -6.05, -166.3), rgba="0.7 0.7 0.75 1")
                 + _vis("elbow_2", (0.8, -6.05, -166.3), rgba="0.7 0.7 0.75 1")) if meshes else \
        f'<geom class="link" type="capsule" size="0.016" fromto="0 0 0 0 0 {L1}"/>'
    fore_vis = (_vis("link_2", (0.75, 1.99, -345.6)) + _vis("wrist_gear_1", (0.75, 1.99, -345.6), rgba="0.7 0.7 0.75 1")) \
        if meshes else f'<geom class="link" type="capsule" size="0.014" fromto="0 0 0 0 0 {L2}"/>'
    hand_vis = _vis("gripper_body", (0, 0, 0), rgba="0.9 0.9 0.92 1") if meshes else ""
    jaw_vis = ('<geom type="mesh" mesh="moving_jaw" pos="0 0.0189 0" euler="-90 0 0" rgba="0.9 0.9 0.92 1" '
               'contype="0" conaffinity="0" mass="0"/>') if meshes else ""
    return f"""
    <body name="base" pos="0 0 {a['base_riser']}">
      <joint name="shoulder_pan" axis="0 0 1" range="{lim['shoulder_pan'][0]} {lim['shoulder_pan'][1]}"/>
      {_vis("base", (0.8, -5.9, 38.4), rgba="0.6 0.6 0.65 1") + _vis("base_plate", (0.8, -5.9, 38.4), rgba="0.4 0.4 0.45 1") if meshes else ""}
      <geom class="link" type="cylinder" size="0.03 {a['shoulder_height']/2}" pos="0 0 {a['shoulder_height']/2}" mass="0.15" rgba="0.6 0.6 0.65 1"/>
      <body name="upper_arm" pos="0 0 {a['shoulder_height']}">
        <joint name="shoulder_lift" axis="0 1 0" range="{lim['shoulder_lift'][0]} {lim['shoulder_lift'][1]}"/>
        <inertial pos="0 0 {L1/2}" mass="{m}" diaginertia="{m*L1*L1/12} {m*L1*L1/12} 1e-5"/>
        {upper_vis}
        <body name="forearm" pos="0 0 {L1}">
          <joint name="elbow_flex" axis="0 1 0" range="{lim['elbow_flex'][0]} {lim['elbow_flex'][1]}"/>
          <inertial pos="0 0 {L2/2}" mass="{m}" diaginertia="{m*L2*L2/12} {m*L2*L2/12} 1e-5"/>
          {fore_vis}
          <body name="hand" pos="0 0 {L2}">
            <joint name="wrist_roll" axis="0 0 1" range="{lim['wrist_roll'][0]} {lim['wrist_roll'][1]}"/>
            <inertial pos="-0.01 0.01 0.05" mass="0.08" diaginertia="5e-5 5e-5 2e-5"/>
            {hand_vis}
            <!-- fixed jaw (part of the SO-101 wrist-roll follower): a box on its real inner face -->
            <body name="fixed_jaw" pos="{fx_} 0 {fz_}" euler="0 {f_tilt} 0">
              <geom name="fixed_pad" class="pad" type="box" size="0.003 0.007 {f_len/2 + 0.003}" pos="-0.003 0 0"
                    rgba="0.25 0.25 0.25 {0 if meshes else 1}"/>
            </body>
            <site name="grasp" pos="{grasp[0]} {grasp[1]} {grasp[2]}" size="0.003" rgba="1 0 0 1"/>
            <site name="tcp_so101" pos="-0.0079 0.0002 0.0981" size="0.002" rgba="0 0 0 0"/>
            {_pusher_xml(a, grasp)}
            <camera name="wrist" pos="{cam['offset'][0]} {cam['offset'][1]} {cam['offset'][2]}" xyaxes="1 0 0 0 -1 0" fovy="{cam['fovy_deg']}"/>
            <site name="wrist_cam" pos="{cam['offset'][0]} {cam['offset'][1]} {cam['offset'][2]}" size="0.002" rgba="0 0 0 0"/>
            {"".join(f'<site name="cam_look_{d}" pos="{cam['offset'][0]} {cam['offset'][1]} {cam['offset'][2] + d / 100}" size="0.002" rgba="0 0 0 0"/>' for d in LOOK_CM)}
            <!-- moving jaw: hinge from the SO-101 URDF; the SensX25 is on its inner face -->
            <body name="moving_jaw" pos="{hx} {hy} {hz}">
              <joint name="gripper" axis="0 1 0" range="{jr[0]} {jr[1]}" damping="0.02" armature="0.0005"/>
              <inertial pos="-0.008 0.019 0.04" mass="0.03" diaginertia="2e-5 2e-5 5e-6"/>
              {jaw_vis}
              <body name="jaw_face" pos="{mx_ - hx} {-hy} {mz_ - hz}" euler="0 {m_tilt} 0">
                <geom name="moving_finger" class="solid" type="box" size="0.003 0.008 {m_len/2 + 0.004}" pos="0.003 0 0"
                      rgba="0.85 0.85 0.85 {0 if meshes else 1}"/>
                {_pads(pitch)}
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>"""


def build_xml(sim: dict, bottle: dict, grip_current_ma: float = 1000) -> str:
    """MJCF for one bottle profile. `sim` = configs/sim.yaml, `bottle` = an entry of bottles.yaml."""
    fx, cam = sim["fixture"], sim["wrist_camera"]
    a = {**sim["arm"], "taxel_pitch": sim["tactile"]["taxel_pitch"], "cap_height": fx["cap_height"]}
    tq = a["torque"]
    body_r = bottle["body_diameter_mm"] / 2000
    cap_r = bottle["cap_diameter_mm"] / 2000
    cap_h = fx["cap_height"]
    body_h = bottle["height_mm"] / 1000 - cap_h
    floor = fx["clip_floor"]
    cx, cy = fx["clip_xy"]
    grip_torque = grip_current_ma * a["grip_torque_per_ma"]
    riser = a["base_riser"]
    stiff = math.radians(a.get("servo_stiffness_deg", 2.5))
    actuators = "".join(
        f'<position name="{j}" joint="{j}" kp="{tq[j] / stiff}" forcerange="{-tq[j]} {tq[j]}"/>'
        for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_roll"))
    return f"""<mujoco model="pillbot">
  <compiler angle="degree" autolimits="true"/>
  <!-- noslip: MuJoCo friction is "soft" and creeps under steady load (a held cap would slowly slide
       out of the jaws, a locked cap would slowly turn). The noslip pass removes that creep. -->
  <option timestep="{sim['physics']['timestep']}" cone="elliptic" impratio="10"
          noslip_iterations="{sim['physics'].get('noslip_iterations', 100)}"/>
  <visual><global offwidth="{cam['width'] * 2}" offheight="{cam['height'] * 2}"/></visual>
  <default>
    <joint damping="0.08" armature="0.005"/>
    <default class="link"><geom contype="0" conaffinity="0" rgba="0.95 0.55 0.15 1"/></default>
    <default class="solid"><geom contype="1" conaffinity="1" friction="1 0.005 0.0001"/></default>
    <default class="pad"><geom contype="1" conaffinity="1" friction="1 0.005 0.0001" mass="0.0005"
                               solref="0.004 1" solimp="0.9 0.97 0.001"/></default>
  </default>
  <asset>
    {MESHES if a.get("meshes", True) else ""}
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.8 0.75" rgb2="0.78 0.76 0.71" width="300" height="300"/>
    <material name="table" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0.2 -0.2 1.0" dir="-0.2 0.2 -1" directional="true" diffuse="0.7 0.7 0.7"/>
    <light pos="0.1 0.3 0.6" diffuse="0.3 0.3 0.3"/>
    <camera name="overview" pos="0.42 -0.38 0.36" mode="targetbody" target="bottle"/>
    <camera name="side" pos="0.10 -0.45 0.12" mode="targetbody" target="bottle"/>
    <geom name="table" type="plane" size="0.6 0.6 0.01" material="table" contype="1" conaffinity="1"/>
    {f'<geom name="riser" type="box" size="{a.get('riser_half_width', 0.075)} {a.get('riser_half_width', 0.075)} {riser/2}" pos="0 0 {riser/2}" rgba="0.4 0.4 0.45 1" contype="1" conaffinity="1"/>' if riser > 0 else ''}
    {_arm_xml(a, cam, cap_r)}
    <!-- dark mat under the clip: a white cap against a light table is hard to see (real robot too) -->
    <geom name="mat" type="box" size="0.09 0.09 0.0005" pos="{cx} {cy} 0.0005" rgba="0.12 0.12 0.14 1" contype="0" conaffinity="0"/>
    <body name="bottle" pos="{cx} {cy} 0">
      <geom name="clip" type="box" size="{body_r + 0.012} {body_r + 0.012} {floor / 2 + 0.012}" pos="0 0 {floor / 2 + 0.012}" rgba="0.25 0.3 0.4 1" contype="0" conaffinity="0"/>
      <geom name="bottle_body" class="solid" type="cylinder" size="{body_r} {body_h/2}" pos="0 0 {floor + body_h/2}" rgba="0.75 0.4 0.08 1"/>
      <body name="cap_thread" pos="0 0 {floor + body_h}">
        <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
        <!-- x/y/lift are locked until the thread lets go; then the cap can be carried anywhere -->
        <joint name="cap_x" type="slide" axis="1 0 0" range="-0.4 0.4" damping="0.2" armature="1.0"/>
        <joint name="cap_y" type="slide" axis="0 1 0" range="-0.4 0.4" damping="0.2" armature="1.0"/>
        <joint name="cap_lift" type="slide" axis="0 0 1" range="-0.2 0.2" damping="0.2" armature="1.0"/>
        <joint name="cap_rot" axis="0 0 1" damping="0.002" armature="0.001"/>
        <body name="cap">
          <joint name="cap_press" type="slide" axis="0 0 1" range="-0.003 0" stiffness="{sim['cap']['push_spring']}" damping="20" armature="0.05"/>
          <!-- armature (added mass) keeps the 10 g cap on a stiff spring stable at a 2 ms timestep -->
          <geom name="cap" class="solid" type="cylinder" size="{cap_r} {cap_h/2}" pos="0 0 {cap_h/2}" mass="{fx['cap_mass']}" rgba="0.97 0.97 0.95 1"/>
          <geom name="cap_mark" type="box" size="{cap_r * 0.8} 0.002 0.0005" pos="0 0 {cap_h}" rgba="0.1 0.4 0.9 1" contype="0" conaffinity="0" mass="0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <contact>
    <!-- the real cap skirt surrounds the neck with room to push down; the joints model that travel -->
    <exclude body1="bottle" body2="cap"/>
  </contact>
  <equality>
    <!-- switched on by SimWorld: hold the cap where it is (child-proof lock / thread not yet free).
         MuJoCo constraint stiffness scales with the inertia of what's held, so the cap joints carry
         armature (added inertia, no weight): otherwise a 10 g cap sags/turns under the push/twist. -->
    <joint name="cap_rot_lock" joint1="cap_rot" polycoef="0 0 0 0 0" active="false" solref="0.004 1"/>
    <joint name="cap_lift_lock" joint1="cap_lift" polycoef="0 0 0 0 0" active="true" solref="0.004 1"/>
    <joint name="cap_x_lock" joint1="cap_x" polycoef="0 0 0 0 0" active="true" solref="0.004 1"/>
    <joint name="cap_y_lock" joint1="cap_y" polycoef="0 0 0 0 0" active="true" solref="0.004 1"/>
  </equality>
  <actuator>
    {actuators}
    <position name="gripper" joint="gripper" kp="{grip_torque / stiff}" forcerange="{-grip_torque} {grip_torque}"/>
  </actuator>
  <sensor>
    {''.join(f'<touch name="taxel_{r}_{c}" site="taxel_{r}_{c}"/>' for r in range(TAXELS) for c in range(TAXELS))}
  </sensor>
</mujoco>"""


def load_sim_config(path: str = "configs/sim.yaml") -> dict:
    return load_yaml(path)


if __name__ == "__main__":
    import sys
    print(build_xml(load_sim_config(), get_bottle(sys.argv[1] if len(sys.argv) > 1 else "standard_amber_40dr")))
