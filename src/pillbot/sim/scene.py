"""Build the MuJoCo model (MJCF XML) of the arm, the clamped bottle and the cap from configs/sim.yaml.

Generated instead of hand-written so arm measurements and bottle profiles (configs/bottles.yaml)
change the model without editing XML. Units: meters, kg, radians.

    python -m pillbot.sim.scene > /tmp/pillbot.xml     # inspect the generated model

Model, top to bottom:
  table (z = 0) -> riser -> base [shoulder_pan] -> upper_arm [shoulder_lift] -> forearm [elbow_flex]
  -> hand [wrist_roll] -> two jaws [jaw_l, jaw_r: slides]   + wrist camera, + 5x5 tactile pad on jaw_r
  bottle (fixed in the clip) -> cap_thread [cap_rot: hinge, cap_lift: slide] -> cap [cap_press: spring]
"""
import math

from pillbot.utils.config import get_bottle, load_yaml

TAXELS = 5
JAW_THICK = 0.004           # jaw plate half-thickness
PAD_THICK = 0.0005          # tactile/TPU pad half-thickness (1 mm pads)


def _pusher_xml(a: dict, hand: float) -> str:
    """Optional printed block between the jaws whose bottom face meets the cap TOP when the jaws are
    centred on the cap: the downward push for child-proof caps then goes straight into the cap,
    instead of through the jaws' side friction (which the twist needs)."""
    if not a.get("palm_pusher"):
        return ""
    bottom = a["grasp_depth"] - a["cap_height"] / 2          # from the finger base
    return (f'<geom name="pusher" class="solid" type="box" size="0.010 0.008 {bottom / 2}" '
            f'pos="0 0 {hand + bottom / 2}" rgba="0.3 0.3 0.35 1" mass="0.005"/>')


def _arm_xml(a: dict, cam: dict) -> str:
    lim = a["limits_deg"]
    L1, L2, hand = a["upper_arm"], a["forearm"], a["hand"]
    fl, fw = a["finger_length"], a["finger_width"]
    half_gap = a["max_jaw_gap"] / 2
    pitch = a.get("taxel_pitch", 0.004)
    m = a["link_mass"]
    pads = []
    for r in range(TAXELS):            # row 0 = nearest the hand, col 0 = -y side
        for c in range(TAXELS):
            z = hand + a["grasp_depth"] + (r - 2) * pitch
            y = (c - 2) * pitch
            pads.append(
                f'<geom name="taxel_{r}_{c}" class="pad" type="box" size="{PAD_THICK} {pitch/2 - 0.0002} {pitch/2 - 0.0002}" '
                f'pos="{-PAD_THICK} {y} {z}" rgba="0.2 0.5 0.9 1"/>'
                f'<site name="taxel_{r}_{c}" type="box" size="{PAD_THICK * 3} {pitch/2} {pitch/2}" pos="{-PAD_THICK} {y} {z}" rgba="0 0 0 0"/>')
    pad_z = hand + a["grasp_depth"]
    side = TAXELS * pitch / 2
    return f"""
    <body name="base" pos="0 0 {a['base_riser']}">
      <joint name="shoulder_pan" axis="0 0 1" range="{lim['shoulder_pan'][0]} {lim['shoulder_pan'][1]}"/>
      <geom class="link" type="cylinder" size="0.035 {a['shoulder_height']/2}" pos="0 0 {a['shoulder_height']/2}" mass="0.15"/>
      <body name="upper_arm" pos="0 0 {a['shoulder_height']}">
        <joint name="shoulder_lift" axis="0 1 0" range="{lim['shoulder_lift'][0]} {lim['shoulder_lift'][1]}"/>
        <geom class="link" type="capsule" size="0.016" fromto="0 0 0 0 0 {L1}" mass="{m}"/>
        <body name="forearm" pos="0 0 {L1}">
          <joint name="elbow_flex" axis="0 1 0" range="{lim['elbow_flex'][0]} {lim['elbow_flex'][1]}"/>
          <geom class="link" type="capsule" size="0.014" fromto="0 0 0 0 0 {L2}" mass="{m}"/>
          <body name="hand" pos="0 0 {L2}">
            <joint name="wrist_roll" axis="0 0 1" range="{lim['wrist_roll'][0]} {lim['wrist_roll'][1]}"/>
            <geom name="palm" class="solid" type="box" size="0.023 0.013 {hand/2}" pos="0 0 {hand/2}" mass="0.04" rgba="0.3 0.3 0.35 1"/>
            <site name="grasp" pos="0 0 {pad_z}" size="0.003" rgba="1 0 0 1"/>
            {_pusher_xml(a, hand)}
            <camera name="wrist" pos="{cam['offset'][0]} {cam['offset'][1]} {cam['offset'][2]}" xyaxes="1 0 0 0 -1 0" fovy="{cam['fovy_deg']}"/>
            <site name="wrist_cam" pos="{cam['offset'][0]} {cam['offset'][1]} {cam['offset'][2]}" size="0.002" rgba="0 0 0 0"/>
            <body name="jaw_r" pos="0 0 {hand}">
              <joint name="jaw_r" type="slide" axis="1 0 0" range="0 {half_gap}" damping="2"/>
              <geom class="solid" type="box" size="{JAW_THICK} {fw/2} {fl/2}" pos="{JAW_THICK} 0 {fl/2}" mass="0.01" rgba="0.85 0.85 0.85 1"/>
              <body name="pad_r" pos="0 0 {-hand}">
                {''.join(pads)}
              </body>
            </body>
            <body name="jaw_l" pos="0 0 {hand}">
              <joint name="jaw_l" type="slide" axis="-1 0 0" range="0 {half_gap}" damping="2"/>
              <geom class="solid" type="box" size="{JAW_THICK} {fw/2} {fl/2}" pos="{-JAW_THICK} 0 {fl/2}" mass="0.01" rgba="0.85 0.85 0.85 1"/>
              <geom name="pad_l" class="pad" type="box" size="{PAD_THICK} {side} {side}" pos="{PAD_THICK} 0 {a['grasp_depth']}" rgba="0.2 0.2 0.2 1"/>
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
    grip_force = grip_current_ma * a["grip_force_per_ma"]
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
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.8 0.75" rgb2="0.78 0.76 0.71" width="300" height="300"/>
    <material name="table" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0.2 -0.2 1.0" dir="-0.2 0.2 -1" directional="true" diffuse="0.7 0.7 0.7"/>
    <light pos="0.1 0.3 0.6" diffuse="0.3 0.3 0.3"/>
    <camera name="overview" pos="0.42 -0.38 0.36" mode="targetbody" target="bottle"/>
    <camera name="side" pos="0.10 -0.45 0.12" mode="targetbody" target="bottle"/>
    <geom name="table" type="plane" size="0.6 0.6 0.01" material="table" contype="1" conaffinity="1"/>
    {f'<geom name="riser" type="box" size="{a.get('riser_half_width', 0.045)} {a.get('riser_half_width', 0.045)} {riser/2}" pos="0 0 {riser/2}" rgba="0.4 0.4 0.45 1" contype="1" conaffinity="1"/>' if riser > 0 else ''}
    {_arm_xml(a, cam)}
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
    <position name="jaw_r" joint="jaw_r" kp="{grip_force / 0.004}" forcerange="{-grip_force} {grip_force}"/>
    <position name="jaw_l" joint="jaw_l" kp="{grip_force / 0.004}" forcerange="{-grip_force} {grip_force}"/>
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
