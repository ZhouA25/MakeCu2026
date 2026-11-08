"""Full PillBot run:
trigger -> find bottle -> pick -> place in clip -> clamp -> grip cap, push down, twist -> drop cap -> home.

  python -m pillbot.run --dry-run                         # simulate everything, no hardware
  python -m pillbot.run                                   # scripted pick + scripted open (config defaults)
  python -m pillbot.run --trigger zone                    # start as soon as a bottle is set down
  python -m pillbot.run --pick policy                     # ACT policy picks/places, script opens
  python -m pillbot.run --pick skip                       # bottle already in the clip
  python -m pillbot.run --zone-st 0.5,0.5                 # skip vision, known zone position
  python -m pillbot.run --bottle auto                     # pick the bottle profile from its measured width
  python -m pillbot.run --loop                            # keep serving bottles until Ctrl-C
  python -m pillbot.run --view                            # watch live at http://<this machine>:8080
"""
import argparse
import logging
import sys

from pillbot.control.bottle_select import check_fits, match_bottle
from pillbot.control.fixture import Fixture
from pillbot.control.open_sequence import OpenFailed, open_cap
from pillbot.control.pick_place import pick_bottle, place_in_clip
from pillbot.control.policy_runner import run_policy_phase
from pillbot.control.poses import load_poses, untaught
from pillbot.control.robot_io import Aborted, RobotIO
from pillbot.perception.scan import camera_on_arm
from pillbot.control.triggers import wait_for_trigger
from pillbot.sensors.hub import BTN_STOP, MockHub, make_hub
from pillbot.sensors.tactile import make_tactile
from pillbot.utils.config import get_bottle, load_yaml

log = logging.getLogger("pillbot")


def find_bottle(cfg, io=None, poses=None):
    """-> (s, t, width_mm) of the bottle in the start zone, or None.
    With the camera on the arm, look from the `scan` pose first."""
    import time
    from pillbot.perception.detect_bottle import make_detector, settings, width_mm
    from pillbot.perception.scan import camera_on_arm, goto_scan
    from pillbot.perception.shared_camera import SharedCamera
    from pillbot.perception.zone import ZoneMapper
    mapper = ZoneMapper.load()
    detector = make_detector(cfg, mapper=mapper)
    if getattr(detector, "background", True) is None:
        raise OpenFailed("no background image — run detect_bottle / live_view and capture it with the zone empty")
    cam = SharedCamera.get(cfg)             # same connection the live viewer uses
    if camera_on_arm(cfg) and io is not None:
        goto_scan(io, poses or load_poses(), cfg)
    after = time.time()                     # only trust frames taken after the arm settled
    st = settings(cfg)
    for _ in range(15):                     # a few frames for a stable detection
        frame = cam.wait_new(after, timeout=3.0)
        if frame is None:
            raise OpenFailed(f"no frames from camera '{cam.name}': {cam.error or 'timeout'}")
        after = cam.latest()[1]
        det = detector(frame)
        if det is None:
            continue
        s, t = mapper.to_zone(*det.anchor_px(st["anchor"]))
        if mapper.inside(s, t):
            return s, t, width_mm(det, mapper, st["zone_size_mm"], st["anchor"])
    return None


def one_cycle(args, cfg, pcfg, io, tactile, hub, fixture, poses, bottle):
    """`bottle`/`poses` may be None when --bottle auto: chosen from the measured width."""
    gcfg = cfg["gripper"]
    home = (poses or load_poses())["home"]
    io.move_to(home | {"gripper": gcfg["open"]})
    tactile.tare()
    if not args.dry_run:
        wait_for_trigger(args.trigger, hub, cfg.get("trigger", {}))

    # 1. pick & place
    if fixture:
        fixture.unclamp()
    if args.pick == "scripted":
        width = None
        if args.zone_st:
            st = tuple(float(x) for x in args.zone_st.split(","))
        elif args.dry_run:
            st, width = (0.5, 0.5), 41.0
        else:
            found = find_bottle(cfg, io, poses)
            if found is None:
                raise OpenFailed("no bottle found in the start zone")
            *st, width = found
        if bottle is None:                                  # --bottle auto
            bottle = match_bottle(width) or get_bottle(cfg.get("default_bottle", "standard_amber_40dr"))
            log.info("bottle ~%s mm wide -> profile %s", f"{width:.0f}" if width else "?", bottle["name"])
            poses = load_poses(bottle=bottle["name"])
            if (warn := check_fits(bottle, gcfg)):
                raise OpenFailed(warn)
        log.info("bottle at zone s=%.2f t=%.2f", *st)
        if args.dry_run:
            io.sim_gripper_block = 35.0           # bottle body is wider than the cap
        if not pick_bottle(io, tactile, poses, gcfg, *st):
            raise OpenFailed("missed the bottle twice — check zone calibration / taught poses")
        place_in_clip(io, poses, gcfg)
    if bottle is None:                                      # auto, but no measurement (policy / skip)
        bottle = get_bottle(cfg.get("default_bottle", "standard_amber_40dr"))
        poses = load_poses(bottle=bottle["name"])
    if args.pick == "policy":
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
            if args.spin_check and camera_on_arm(cfg):
                log.warning("--spin-check needs a fixed camera; the wrist camera turns with the cap — skipped")
            elif args.spin_check and not args.dry_run:
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
    ap.add_argument("--bottle", default="standard_amber_40dr",
                    help="profile from bottles.yaml, or 'auto' = pick by the measured width")
    ap.add_argument("--pick", choices=["scripted", "policy", "skip"], default="scripted")
    ap.add_argument("--open", choices=["scripted", "policy"], default="scripted", dest="open_mode")
    ap.add_argument("--trigger", choices=["enter", "button", "zone"], help="default: config trigger.default")
    ap.add_argument("--zone-st", help="s,t in [0,1] — bypass camera detection")
    ap.add_argument("--spin-check", action="store_true", help="watch blue tape on the bottle for spinning in the clamp")
    ap.add_argument("--loop", action="store_true", help="repeat until Ctrl-C")
    ap.add_argument("--view", action="store_true", help="live viewer at http://<this machine>:8080 during the run")
    ap.add_argument("--view-port", type=int, default=8080)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")

    cfg, pcfg = load_yaml(args.config), load_yaml("configs/policies.yaml")
    args.trigger = args.trigger or cfg.get("trigger", {}).get("default", "enter")
    if args.bottle == "auto":
        bottle, poses = None, None
        check_poses = load_poses()
    else:
        bottle = get_bottle(args.bottle)
        poses = check_poses = load_poses(bottle=bottle["name"])
        if (warn := check_fits(bottle, cfg["gripper"])):
            sys.exit(warn)
    if not args.dry_run and (missing := untaught(check_poses)):
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
    view = None
    if args.view and not args.dry_run:
        from pillbot.tools.live_view import start_live_view
        view = start_live_view(cfg, args.view_port, tactile)
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
                io.move_to(check_poses["home"])
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
        if view:
            view.stop()
        from pillbot.perception.shared_camera import SharedCamera
        for cam in list(SharedCamera._instances.values()):
            cam.close()
        if fixture:
            fixture.power(False)
        tactile.close()
        hub.close()
        io.disconnect()
    sys.exit(code)


if __name__ == "__main__":
    main()
