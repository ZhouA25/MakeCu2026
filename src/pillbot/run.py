"""Full PillBot run:
trigger -> find bottle -> pick -> place in clip -> clamp -> grip cap, push down, twist -> drop cap -> home.

  python -m pillbot.run --dry-run                         # simulate everything, no hardware
  python -m pillbot.run                                   # scripted pick + scripted open (config defaults)
  python -m pillbot.run --trigger zone                    # start as soon as a bottle is set down
  python -m pillbot.run --pick policy                     # ACT policy picks/places, script opens
  python -m pillbot.run --pick skip                       # bottle already in the clip
  python -m pillbot.run --zone-st 0.5,0.5                 # skip vision, known zone position
  python -m pillbot.run --loop                            # keep serving bottles until Ctrl-C
"""
import argparse
import logging
import sys

from pillbot.control.fixture import Fixture
from pillbot.control.open_sequence import OpenFailed, open_cap
from pillbot.control.pick_place import pick_bottle, place_in_clip
from pillbot.control.policy_runner import run_policy_phase
from pillbot.control.poses import load_poses, untaught
from pillbot.control.robot_io import Aborted, RobotIO
from pillbot.control.triggers import wait_for_trigger
from pillbot.sensors.hub import BTN_STOP, MockHub, make_hub
from pillbot.sensors.tactile import make_tactile
from pillbot.utils.config import get_bottle, load_yaml

log = logging.getLogger("pillbot")


def find_bottle(cfg) -> tuple[float, float] | None:
    from pillbot.perception.cameras import grab, perception_camera
    from pillbot.perception.detect_bottle import locate_in_zone
    from pillbot.perception.zone import ZoneMapper
    mapper = ZoneMapper.load()
    cap = perception_camera(cfg)
    try:
        for _ in range(15):                       # a few frames for a stable detection
            st = locate_in_zone(grab(cap), mapper)
            if st:
                return st
    finally:
        cap.release()
    return None


def one_cycle(args, cfg, pcfg, io, tactile, hub, fixture, poses, bottle):
    gcfg = cfg["gripper"]
    io.move_to(poses["home"] | {"gripper": gcfg["open"]})
    tactile.tare()
    if not args.dry_run:
        wait_for_trigger(args.trigger, hub, cfg.get("trigger", {}))

    # 1. pick & place
    if fixture:
        fixture.unclamp()
    if args.pick == "scripted":
        if args.zone_st:
            st = tuple(float(x) for x in args.zone_st.split(","))
        elif args.dry_run:
            st = (0.5, 0.5)
        else:
            st = find_bottle(cfg)
        if st is None:
            raise OpenFailed("no bottle found in the start zone")
        log.info("bottle at zone s=%.2f t=%.2f", *st)
        if args.dry_run:
            io.sim_gripper_block = 35.0           # bottle body is wider than the cap
        if not pick_bottle(io, tactile, poses, gcfg, *st):
            raise OpenFailed("missed the bottle twice — check zone calibration / taught poses")
        place_in_clip(io, poses, gcfg)
    elif args.pick == "policy":
        if not run_policy_phase(io, cfg, pcfg, "pick_place", dry_run=args.dry_run):
            raise OpenFailed("pick_place policy failed")

    if fixture and not args.dry_run and cfg.get("hub", {}).get("enabled"):
        if hub.state.clip_mm < 0:
            log.warning("no clip ultrasonic reading — skipping the 'bottle seated' check")
        elif not fixture.bottle_seated():
            raise OpenFailed("clip ultrasonic doesn't see a seated bottle")

    # 2. open
    if args.dry_run:
        io.sim_gripper_block = 15.0               # cap
    if fixture:
        fixture.clamp()
    try:
        if args.open_mode == "policy":
            ok = run_policy_phase(io, cfg, pcfg, "open_cap", dry_run=args.dry_run)
            log.info("open_cap policy finished: %s", ok)
        else:
            monitor, monitor_cap = None, None
            if args.spin_check and not args.dry_run:
                from pillbot.perception.cameras import perception_camera
                from pillbot.perception.spin_monitor import BottleSpinMonitor
                monitor_cap = perception_camera(cfg)
                monitor = BottleSpinMonitor(monitor_cap)
                monitor.arm()
            try:
                stats = open_cap(io, tactile, bottle, poses, cfg["gripper"], monitor=monitor, ocfg=cfg.get("open"))
                log.info("opened! %s", stats)
            finally:
                if monitor_cap is not None:
                    monitor_cap.release()
    finally:
        if fixture:
            fixture.unclamp()
    io.move_to(poses["home"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--bottle", default="standard_amber_40dr")
    ap.add_argument("--pick", choices=["scripted", "policy", "skip"], default="scripted")
    ap.add_argument("--open", choices=["scripted", "policy"], default="scripted", dest="open_mode")
    ap.add_argument("--trigger", choices=["enter", "button", "zone"], help="default: config trigger.default")
    ap.add_argument("--zone-st", help="s,t in [0,1] — bypass camera detection")
    ap.add_argument("--spin-check", action="store_true", help="watch blue tape on the bottle for spinning in the clamp")
    ap.add_argument("--loop", action="store_true", help="repeat until Ctrl-C")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")

    cfg, pcfg = load_yaml(args.config), load_yaml("configs/policies.yaml")
    args.trigger = args.trigger or cfg.get("trigger", {}).get("default", "enter")
    poses, bottle = load_poses(), get_bottle(args.bottle)
    if not args.dry_run and (missing := untaught(poses)):
        sys.exit(f"Untaught poses: {missing}\nRun: python -m pillbot.control.teach --missing")

    hub = make_hub(cfg, dry_run=args.dry_run)
    if args.trigger in ("button", "zone") and isinstance(hub, MockHub) and not args.dry_run:
        sys.exit(f"--trigger {args.trigger} needs the hub (set hub.enabled: true in configs/robot.yaml)")
    hub_on = bool(cfg.get("hub", {}).get("enabled")) or args.dry_run
    if not hub_on:
        log.warning("hub disabled: no clamp — the clip alone must stop the bottle turning")
    fixture = Fixture(hub, cfg["fixture"], dry_run=args.dry_run) if hub_on else None

    io = RobotIO.from_config(cfg, dry_run=args.dry_run)
    hub.connect()
    tactile = make_tactile(cfg, io, hub=hub, dry_run=args.dry_run)
    io.connect()
    tactile.connect()
    io.should_stop = lambda: hub.state.pressed(BTN_STOP)
    if fixture:
        fixture.power(True)
    code = 0
    try:
        while True:
            try:
                one_cycle(args, cfg, pcfg, io, tactile, hub, fixture, poses, bottle)
            except OpenFailed as e:
                code = 1
                log.error("failed: %s", e)
                io.set_gripper(cfg["gripper"]["open"])
                io.move_to(poses["home"])
            if not args.loop:
                break
    except Aborted:
        code = 2
        log.error("STOP pressed — fixture power cut, arm holding position.")
        if not args.dry_run:
            input("Support the arm, then press Enter to release torque...")
    except KeyboardInterrupt:
        log.warning("stopped by user")
    finally:
        if fixture:
            fixture.power(False)
        tactile.close()
        hub.close()
        io.disconnect()
    sys.exit(code)


if __name__ == "__main__":
    main()
