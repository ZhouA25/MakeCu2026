"""The bottle fixture: a funnel clip with a servo clamp (goBILDA 2000 #1) that holds the bottle body
still while the arm pushes down on the cap and twists it off with its wrist.
"""
import logging
import time

log = logging.getLogger(__name__)


class Fixture:
    def __init__(self, hub, fcfg: dict, dry_run: bool = False):
        self.hub, self.c, self.dry_run = hub, fcfg, dry_run
        self.clamped = False

    # --- power (hub relay -> hobby-servo rail) -----------------------------------
    def power(self, on: bool):
        self.hub.relay(on)
        if on and not self.dry_run:
            time.sleep(0.2)

    # --- clamp -------------------------------------------------------------------
    def clamp(self):
        self.hub.servo_us(self.c["clamp_channel"], self.c["clamp_closed_us"])
        self.clamped = True
        self._wait(self.c.get("clamp_settle_s", 0.6))

    def unclamp(self):
        self.hub.servo_us(self.c["clamp_channel"], self.c["clamp_open_us"])
        self.clamped = False
        self._wait(self.c.get("clamp_settle_s", 0.6))

    def release_servo(self):
        """Let the clamp servo go limp (no holding current)."""
        self.hub.servo_us(self.c["clamp_channel"], 0)

    # --- sensing -----------------------------------------------------------------
    def bottle_seated(self) -> bool:
        mm = self.hub.state.clip_mm
        return 0 < mm <= self.c.get("clip_seated_mm", 60)

    def _wait(self, s):
        if not self.dry_run:
            time.sleep(s)
