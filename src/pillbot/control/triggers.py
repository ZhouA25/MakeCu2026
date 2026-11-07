"""Ways for the user to say "open my bottle" — chosen for people with limited grip/dexterity.

  enter   press Enter on the laptop
  button  press the big START button
  zone    just put the bottle in the start zone (ultrasonic sees it) — no button press needed
"""
import logging
import time

from pillbot.sensors.hub import BTN_START

log = logging.getLogger(__name__)


def wait_for_trigger(kind: str, hub, tcfg: dict, timeout: float | None = None) -> bool:
    if kind == "enter":
        input("Ready — press Enter to start (Ctrl-C to stop)...")
        return True
    log.info("waiting for trigger: %s", kind)
    t0 = time.time()
    held_since = None
    while timeout is None or time.time() - t0 < timeout:
        s = hub.state
        if kind == "button":
            hit = s.pressed(BTN_START)
        elif kind == "zone":
            hit = 0 < s.zone_mm <= tcfg.get("zone_present_mm", 120)
        else:
            raise ValueError(f"unknown trigger {kind}")
        if hit:
            held_since = held_since or time.time()
            if time.time() - held_since >= tcfg.get("hold_s", 0.3):   # debounce
                log.info("trigger: %s", kind)
                return True
        else:
            held_since = None
        time.sleep(0.02)
    return False

