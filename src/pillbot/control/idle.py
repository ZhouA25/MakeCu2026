"""Keep the robot at rest and drawing little current while it isn't doing anything.

Between runs (waiting for the trigger) and when the program ends, `go_idle`:
  1. moves the arm to the `rest` pose (falls back to `home` if `rest` isn't taught) and opens the
     gripper, so nothing is being squeezed;
  2. turns torque OFF on the joints in `idle.limp` — by default the ones gravity doesn't pull on
     (shoulder_pan turns about a vertical axis, wrist_roll about the forearm, the gripper holds nothing),
     so they can't sag but stop drawing holding current. `limp: all` also relaxes shoulder_lift and
     elbow_flex: ONLY if the rest pose is physically supported (arm resting on a stand), or it drops;
  3. lets the clamp servo go limp (no PWM) and, with `servo_power_off`, cuts the hub relay — unless a
     hub-servo joint (the goBILDA elbow) is still holding the arm up;
  4. reads each motor's current and warns about any joint still drawing more than `max_current_ma`
     (teach a more balanced rest pose: forearm upright over the shoulder, or support it).
Any later motion wakes the limp joints where they are (RobotIO.torque_on), so nothing jumps.

Teach the rest pose:  python -m pillbot.control.teach --name rest
"""
import logging

log = logging.getLogger(__name__)

DEFAULT_IDLE = {
    "pose": "rest",
    "limp": ["shoulder_pan", "wrist_roll", "gripper"],
    "servo_power_off": True,
    "max_current_ma": 150,
}


def idle_cfg(cfg: dict) -> dict:
    return {**DEFAULT_IDLE, **(cfg.get("idle") or {})}


def rest_pose(poses: dict, name: str = "rest") -> dict:
    p = poses.get(name)
    if p and any(p.values()):
        return p
    log.debug("idle: pose '%s' not taught, resting at home", name)
    return poses["home"]


def go_idle(io, poses: dict, cfg: dict, fixture=None) -> dict | None:
    """Park the robot. Returns per-joint current (mA) after relaxing, or None (dry run / no reading)."""
    icfg = idle_cfg(cfg)
    io.move_to(rest_pose(poses, icfg["pose"]) | {"gripper": cfg["gripper"]["open"]})
    limp = list(io.joints) if icfg["limp"] == "all" else list(icfg["limp"] or [])
    io.torque_off(limp)
    if fixture is not None:
        fixture.release_servo()
        held = [j for j in getattr(io, "pwm", {}) if j not in io.limp]
        if icfg["servo_power_off"] and held:
            log.debug("idle: servo rail stays on — %s (hub servo) is holding the arm", held)
        elif icfg["servo_power_off"]:
            fixture.power(False)
    load = io.read_load()
    if load:
        high = {j: round(abs(v)) for j, v in load.items() if j not in io.limp and abs(v) > icfg["max_current_ma"]}
        if high:
            log.warning("idle: still drawing %s mA at rest — teach a more balanced `rest` pose "
                        "(forearm upright over the shoulder) or support the arm", high)
        log.info("idle: resting (limp: %s), total %.0f mA", ", ".join(sorted(io.limp)) or "none",
                 sum(abs(v) for v in load.values()))
    return load


def wake(io, fixture=None, cfg: dict | None = None):
    """Back to work: energise limp joints where they are, power the servo rail again."""
    io.torque_on()
    if fixture is not None and idle_cfg(cfg or {})["servo_power_off"]:
        fixture.power(True)
