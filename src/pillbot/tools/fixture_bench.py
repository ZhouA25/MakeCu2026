"""Fixture bench test — exercise the hub, clip and clamp WITHOUT the arm.

  python -m pillbot.tools.fixture_bench status              live readout: ultrasonics + buttons (Ctrl-C to quit)
  python -m pillbot.tools.fixture_bench servo 0 1500        raw pulse on a hub servo channel (0 = detach)
  python -m pillbot.tools.fixture_bench tune clamp          find clamp open/closed pulses, save to config
  python -m pillbot.tools.fixture_bench tune elbow          step the goBILDA elbow to find its safe limits
                                                           (+ direction), save to pwm_joints.elbow_flex
  python -m pillbot.tools.fixture_bench seat                calibrate "bottle seated" distance, save to config
  python -m pillbot.tools.fixture_bench cycle               clamp -> hold -> unclamp, a few times
  python -m pillbot.tools.fixture_bench hand-open           clamp, then YOU push + twist the cap off by hand:
                                                           proves the clamp holds the bottle against a real cap

Add --dry-run to any command to try it with a simulated hub, and -y to skip the Enter prompts.
STOP on the hub aborts at any time (the firmware cuts servo power instantly).
Saved values go into configs/robot.local.yaml if it exists, otherwise configs/robot.yaml
(only the `fixture:` lines change; comments are kept).
"""
import argparse
import re
import sys
import time

from pillbot.control.fixture import Fixture
from pillbot.control.robot_io import Aborted
from pillbot.sensors.hub import BTN_MODE, BTN_START, BTN_STOP, MockHub, make_hub
from pillbot.utils.config import ROOT, get_bottle, load_yaml


# ------------------------------------------------------------------ helpers
def active_config_path():
    local = ROOT / "configs/robot.local.yaml"
    return local if local.exists() else ROOT / "configs/robot.yaml"


def save_fixture_values(values: dict, path=None) -> list[str]:
    """Rewrite `  key: value` lines inside the `fixture:` block, keeping comments. Returns changed keys."""
    path = path or active_config_path()
    lines = path.read_text().splitlines(keepends=True)
    in_block, changed = False, []
    for i, line in enumerate(lines):
        if re.match(r"^fixture:\s*(#.*)?$", line):
            in_block = True
            continue
        if in_block and re.match(r"^\S", line):          # next top-level key ends the block
            in_block = False
        if not in_block:
            continue
        m = re.match(r"^(\s+)(\w+):(\s*)([^#\n]*?)(\s*#.*)?(\n?)$", line)
        if m and m.group(2) in values:
            indent, key, sp, _, comment, nl = m.groups()
            lines[i] = f"{indent}{key}:{sp or ' '}{values[key]}{comment or ''}{nl}"
            changed.append(key)
    path.write_text("".join(lines))
    return changed


def save_block_values(block: list[str], values: dict, path) -> list[str]:
    """Like save_fixture_values for any nested block, e.g. block=["pwm_joints", "elbow_flex"]:
    rewrites `key: value` lines directly inside it, keeping comments. Returns changed keys."""
    lines = path.read_text().splitlines(keepends=True)
    depth, changed, indent_of = 0, [], []
    for i, line in enumerate(lines):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        ind = len(line) - len(line.lstrip())
        while indent_of and ind <= indent_of[-1]:        # left the block we were in
            indent_of.pop()
            depth -= 1
        key = line.strip().split(":", 1)[0]
        if depth < len(block) and key == block[depth] and line.rstrip().split("#")[0].rstrip().endswith(":"):
            indent_of.append(ind)
            depth += 1
            continue
        if depth == len(block):
            m = re.match(r"^(\s+)(\w+):(\s*)([^#\n]*?)(\s*#.*)?(\n?)$", line)
            if m and m.group(2) in values:
                ind_s, k, sp, _, comment, nl = m.groups()
                lines[i] = f"{ind_s}{k}:{sp or ' '}{values[k]}{comment or ''}{nl}"
                changed.append(k)
    path.write_text("".join(lines))
    return changed


class Bench:
    def __init__(self, args):
        self.args = args
        self.cfg = load_yaml(args.config)
        hub_cfg = self.cfg.get("hub", {}) or {}
        if not args.dry_run and not hub_cfg.get("enabled"):
            sys.exit("hub.enabled is false in your config — set it true and set hub.port "
                     "(or use --dry-run)")
        self.hub = make_hub(self.cfg, dry_run=args.dry_run)
        self.fx = Fixture(self.hub, self.cfg["fixture"], dry_run=args.dry_run)

    def __enter__(self):
        self.hub.connect()
        self.fx.power(True)
        return self

    def __exit__(self, *exc):
        try:
            self.fx.power(False)
        finally:
            self.hub.close()

    def ask(self, prompt: str, default: str = "") -> str:
        if self.args.yes or self.args.dry_run:
            print(f"{prompt} [{default or 'auto'}]")
            return default
        return input(prompt + " ").strip() or default

    def pause(self, prompt: str):
        self.ask(prompt + " (Enter)")

    def check_stop(self):
        if self.hub.state.pressed(BTN_STOP):
            raise Aborted("STOP pressed")


# ------------------------------------------------------------------ commands
def cmd_status(b: Bench):
    names = {BTN_START: "START", BTN_STOP: "STOP", BTN_MODE: "MODE"}
    print("clip_mm  zone_mm  seated  buttons      (Ctrl-C to quit)")
    n = 0
    try:
        while True:
            s = b.hub.state
            pressed = ",".join(v for k, v in names.items() if s.pressed(k)) or "-"
            print(f"\r{s.clip_mm:7d}  {s.zone_mm:7d}  {str(b.fx.bottle_seated()):6s}  {pressed:12s}",
                  end="", flush=True)
            time.sleep(0.1)
            n += 1
            if b.args.dry_run and n >= 5:
                break
    except KeyboardInterrupt:
        pass
    print()


def cmd_servo(b: Bench):
    b.hub.servo_us(b.args.channel, b.args.us)
    print(f"servo ch{b.args.channel} -> {b.args.us} us" + ("  (detached)" if b.args.us == 0 else ""))
    if not b.args.dry_run:
        b.pause("Holding. Press Enter to release and power off")


def _tune_pulse(b: Bench, ch: int, label: str, start: int) -> int:
    """Interactive: type a pulse (500-2500), +N / -N to nudge, Enter to accept."""
    us = start
    b.hub.servo_us(ch, us)
    print(f"\n{label}: now {us} us.  Type a value (500-2500), +N / -N to nudge, Enter to accept.")
    while True:
        b.check_stop()
        r = b.ask(f"  {label} [{us}] >", "")
        if r == "":
            return us
        try:
            us = us + int(r) if r[0] in "+-" else int(r)
        except ValueError:
            print("  enter a number like 1600, +20 or -50")
            continue
        us = max(500, min(2500, us))
        b.hub.servo_us(ch, us)
        print(f"  -> {us} us")


def _step_to_limit(b: Bench, ch: int, start: int, direction: int, step: int) -> int | None:
    """Interactive: each Enter moves `step` us further; f = finer steps, b = back one step,
    m = mark here as the limit (just short of touching), q = cancel. Returns the marked pulse."""
    us = start
    side = "BIGGER" if direction > 0 else "SMALLER"
    print(f"\nStepping toward {side} pulses from {us} us. Watch the forearm.\n"
          "  Enter = step   f = finer steps   b = back one step   m = mark limit HERE   q = cancel\n"
          "  Stop the moment anything is about to touch. Servo buzzing = stalled: press b at once.")
    n = 0
    while True:
        b.check_stop()
        n += 1
        r = b.ask(f"  {us} us (step {step}) >", ("m" if n > 4 else "") if b.args.dry_run else "").lower()
        if r == "":
            nxt = us + direction * step
            if not 500 <= nxt <= 2500:
                print("  at the servo's end of travel (500..2500 us) — mark it with m")
                continue
            us = nxt
        elif r == "f":
            step = max(5, step // 2)
            print(f"  step now {step} us")
            continue
        elif r == "b":
            us -= direction * step
        elif r == "m":
            return us
        elif r == "q":
            return None
        else:
            print("  Enter, f, b, m or q")
            continue
        b.hub.servo_us(ch, us)


def cmd_tune(b: Bench):
    if b.args.what == "elbow":
        return cmd_tune_elbow(b)
    c = b.cfg["fixture"]
    if b.args.what == "clamp":
        ch = c["clamp_channel"]
        print("Put an EMPTY bottle in the clip. Fingers out of the jaws.")
        open_us = _tune_pulse(b, ch, "clamp OPEN (bottle drops in freely)", c["clamp_open_us"])
        closed_us = _tune_pulse(b, ch, "clamp CLOSED (bottle can't twist by hand, not crushed)",
                                c["clamp_closed_us"])
        b.hub.servo_us(ch, open_us)
        new = {"clamp_open_us": open_us, "clamp_closed_us": closed_us}
    _maybe_save(b, new)


def cmd_seat(b: Bench):
    b.pause("Clip EMPTY")
    empty = _avg(b, "clip_mm")
    b.pause("Bottle SEATED in the clip")
    seated = _avg(b, "clip_mm")
    print(f"empty: {empty} mm   seated: {seated} mm")
    if seated <= 0:
        print("No echo with a bottle in — aim the ultrasonic at the bottle body, check ECHO divider.")
        return
    if 0 < empty <= seated + 10:
        print("Empty and seated readings are too close — move the sensor so it sees the bottle body.")
        return
    threshold = int(seated + (min(empty, seated + 60) - seated) / 2) if empty > 0 else seated + 20
    print(f"-> clip_seated_mm: {threshold}")
    _maybe_save(b, {"clip_seated_mm": threshold})


def _avg(b: Bench, field: str, n: int = 15) -> int:
    vals = []
    for _ in range(n):
        v = getattr(b.hub.state, field)
        if v > 0:
            vals.append(v)
        time.sleep(0 if b.args.dry_run else 0.05)
    return int(sorted(vals)[len(vals) // 2]) if vals else -1


def cmd_cycle(b: Bench):
    print(f"bottle seated: {b.fx.bottle_seated()}  (clip ultrasonic {b.hub.state.clip_mm} mm)")
    for i in range(b.args.times):
        t0 = time.time()
        b.fx.clamp()
        b.check_stop()
        time.sleep(0 if b.args.dry_run else 0.5)
        b.fx.unclamp()
        b.check_stop()
        print(f"  cycle {i + 1}: clamp + unclamp ok ({time.time() - t0:.1f}s)")
    b.fx.release_servo()
    print("cycle complete")


def cmd_hand_open(b: Bench):
    bottle = get_bottle(b.args.bottle)
    push = bottle["cap_type"] == "push_and_turn"
    print(f"Bottle profile: {bottle['name']} ({bottle['cap_type']}, twist {bottle['twist_deg']}°)")
    print("SAFETY: keep fingers away from the clamp jaws. STOP releases the clamp.")
    b.pause("Place the bottle in the clip")
    b.fx.clamp()
    b.pause("Clamped. Now " + ("PUSH DOWN and " if push else "") +
            "twist the cap off by hand, one hand only — don't hold the bottle. Done?")
    held = b.ask("Did the bottle stay still in the clamp the whole time? [Y/n]", "y").lower()
    deg = b.ask(f"Roughly how many degrees did the cap turn before it came free? [{bottle['twist_deg']}]",
                str(bottle["twist_deg"]))
    b.fx.unclamp()
    b.fx.release_servo()
    if held.startswith("n"):
        print("-> Clamp slipped: raise clamp_closed_us a little (`tune clamp`) and line the jaws with TPU/rubber.")
    else:
        print("-> Clamp holds. The arm only has to grip, push and twist.")
    try:
        deg_i = int(float(deg))
    except ValueError:
        return
    if deg_i != bottle["twist_deg"] and not (b.args.yes or b.args.dry_run):
        if b.ask(f"Set twist_deg for '{bottle['name']}' to {deg_i} in bottles.yaml? [y/N]", "n").lower().startswith("y"):
            _save_twist(bottle["name"], deg_i)


def _save_twist(name: str, deg: int):
    path = ROOT / "configs/bottles.yaml"
    lines, in_bottle = path.read_text().splitlines(keepends=True), False
    for i, line in enumerate(lines):
        if re.match(rf"^\s*-\s*name:\s*{re.escape(name)}\s*$", line):
            in_bottle = True
        elif in_bottle and re.match(r"^\s*-\s*name:", line):
            break
        elif in_bottle and re.match(r"^\s+twist_deg:", line):
            lines[i] = re.sub(r"(twist_deg:\s*)\d+", rf"\g<1>{deg}", line)
            break
    path.write_text("".join(lines))
    print(f"saved twist_deg: {deg} for {name}")


def cmd_tune_elbow(b: Bench):
    """Find the elbow servo's safe pulse range (and which way is +) by stepping it, then save."""
    pj = (b.cfg.get("pwm_joints") or {}).get("elbow_flex")
    if not pj:
        sys.exit("no pwm_joints.elbow_flex in your config")
    ch, start, margin = int(pj["channel"]), int(pj.get("center_us", 1500)), b.args.margin
    print(f"Elbow servo on hub channel {ch}. Fit the whole arm (wrist + gripper on) and put shoulder_lift\n"
          "where you'll use it (forearm near vertical for top-down grasps).\n"
          f"HOLD THE FOREARM: it snaps to {start} us now.")
    b.pause("Ready")
    b.hub.servo_us(ch, start)
    try:
        hi = _step_to_limit(b, ch, start, +1, b.args.step)
        if hi is None:
            return
        b.hub.servo_us(ch, start)
        sign_same = b.ask("\nWhen the pulse got BIGGER, is that the way `e+5` should bend the elbow "
                          "while teaching? [Y/n]", "y").lower() != "n"
        lo = _step_to_limit(b, ch, start, -1, b.args.step)
        if lo is None:
            return
    finally:
        b.hub.servo_us(ch, start)                       # park in the middle, still holding
    new = {"min_us": lo + margin, "max_us": hi - margin, "sign": 1 if sign_same else -1}
    if new["min_us"] >= new["max_us"]:
        sys.exit(f"range too small after the {margin} us margin ({lo}..{hi}) — check the horn position")
    deg = (new["max_us"] - new["min_us"]) / float(pj.get("us_per_deg", 6.67))
    print(f"\ntouch points {lo} / {hi} us, minus {margin} us margin -> about {deg:.0f} deg of safe travel")
    print("pwm_joints:\n  elbow_flex:\n" + "".join(f"    {k}: {v}\n" for k, v in new.items()))
    if b.args.dry_run:
        print("(dry run — not saved)")
        return
    if not b.ask("Save? [y/N]", "n").lower().startswith("y"):
        return
    left, saved = dict(new), []
    for path in (ROOT / "configs/robot.local.yaml", ROOT / "configs/robot.yaml"):
        if left and path.exists():
            done = save_block_values(["pwm_joints", "elbow_flex"], left, path)
            saved += [f"{k} -> {path.relative_to(ROOT)}" for k in done]
            left = {k: v for k, v in left.items() if k not in done}
    print("saved:", ", ".join(saved) or "nothing (no pwm_joints.elbow_flex block found)")
    print("Re-teach any poses taught with the old limits/sign.")


def _maybe_save(b: Bench, values: dict):
    path = active_config_path()
    print("\nfixture:\n" + "".join(f"  {k}: {v}\n" for k, v in values.items()))
    if b.args.dry_run:
        print("(dry run — not saved)")
        return
    if b.ask(f"Save to {path.relative_to(ROOT)}? [y/N]", "n").lower().startswith("y"):
        print("saved:", ", ".join(save_fixture_values(values, path)))


# ------------------------------------------------------------------ main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--dry-run", action="store_true", help="simulated hub")
    ap.add_argument("-y", "--yes", action="store_true", help="don't wait for Enter")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sp = sub.add_parser("servo")
    sp.add_argument("channel", type=int, choices=range(4))
    sp.add_argument("us", type=int)
    tp = sub.add_parser("tune")
    tp.add_argument("what", choices=["clamp", "elbow"])
    tp.add_argument("--step", type=int, default=50, help="elbow: us per Enter press (50 us ~ 7 deg)")
    tp.add_argument("--margin", type=int, default=40, help="elbow: us kept back from each touch point")
    sub.add_parser("seat")
    cp = sub.add_parser("cycle")
    cp.add_argument("--times", type=int, default=3)
    hp = sub.add_parser("hand-open")
    hp.add_argument("--bottle", default="standard_amber_40dr")
    args = ap.parse_args(argv)
    if args.cmd == "servo" and args.us != 0 and not 500 <= args.us <= 2500:
        ap.error("us must be 0 (detach) or 500..2500")

    cmds = {"status": cmd_status, "servo": cmd_servo, "tune": cmd_tune, "seat": cmd_seat,
            "cycle": cmd_cycle, "hand-open": cmd_hand_open}
    try:
        with Bench(args) as b:
            if isinstance(b.hub, MockHub):
                b.hub.state.clip_mm = 40          # pretend a bottle is seated
            cmds[args.cmd](b)
    except Aborted:
        print("\nSTOP pressed — servo power cut by the hub.")
        sys.exit(2)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
