"""The bottle fixture: clip on a servo turntable (goBILDA 2000 #1) with a servo clamp (goBILDA #2 / D646WP).

Turntable trick: instead of twisting the cap with the wrist (90° per grip, many regrips), the arm
grips + presses the cap and holds still, and the turntable spins the BOTTLE underneath —
up to ~300° per pass with 20+ kg·cm of torque.

goBILDA 2000 (position mode): 500–2500 µs, 0.15°/µs  ->  center 1500 µs, ±150°.
"""
import logging
import time

log = logging.getLogger(__name__)


class Fixture:
    def __init__(self, hub, fcfg: dict, dry_run: bool = False):
        self.hub, self.c, self.dry_run = hub, fcfg, dry_run
        self.angle = 0.0                    # current turntable angle, deg (0 = home)

    # --- power -------------------------------------------------------------------
    def power(self, on: bool):
        self.hub.relay(on)
        if on and not self.dry_run:
            time.sleep(0.2)

    # --- clamp -------------------------------------------------------------------
    def clamp(self):
        self.hub.servo_us(self.c["clamp_channel"], self.c["clamp_closed_us"])
        self._wait(self.c.get("clamp_settle_s", 0.6))

    def unclamp(self):
        self.hub.servo_us(self.c["clamp_channel"], self.c["clamp_open_us"])
        self._wait(self.c.get("clamp_settle_s", 0.6))

    # --- turntable ---------------------------------------------------------------
    @property
    def max_angle(self) -> float:
        return self.c.get("turntable_range_deg", 300) / 2

    def _us(self, angle: float) -> int:
        return int(self.c.get("turntable_center_us", 1500) + angle / self.c.get("turntable_deg_per_us", 0.15))

    def rotate_to(self, angle: float, speed_dps: float | None = None):
        """Ramp the turntable to `angle` deg (clamped to its range) at speed_dps."""
        angle = max(-self.max_angle, min(self.max_angle, angle))
        speed = speed_dps or self.c.get("turntable_speed_dps", 90)
        ch, step_dt = self.c["turntable_channel"], 0.02
        while abs(angle - self.angle) > 0.5:
            d = max(-speed * step_dt, min(speed * step_dt, angle - self.angle))
            self.angle += d
            self.hub.servo_us(ch, self._us(self.angle))
            self._wait(step_dt)
        self.angle = angle
        self.hub.servo_us(ch, self._us(angle))

    def home(self):
        self.rotate_to(0)

    def loosen_stroke(self) -> float:
        """Degrees available for one loosening pass, starting from the pre-wound end."""
        return 2 * self.max_angle

    def prewind(self):
        """Move to the end of travel opposite the loosening direction."""
        self.rotate_to(-self._loosen_sign() * self.max_angle)

    def loosen(self, deg: float):
        """Rotate the BOTTLE so the cap unscrews (cap held still by the arm)."""
        self.rotate_to(self.angle + self._loosen_sign() * deg, self.c.get("twist_speed_dps", 45))

    def _loosen_sign(self) -> int:
        # Viewed from above, unscrewing = cap CCW relative to bottle = bottle CW under a fixed cap.
        # Flip `loosen_direction` in config if your servo is mounted the other way.
        return 1 if self.c.get("loosen_direction", "cw") == "cw" else -1

    # --- sensing -----------------------------------------------------------------
    def bottle_seated(self) -> bool:
        mm = self.hub.state.clip_mm
        return 0 < mm <= self.c.get("clip_seated_mm", 60)

    def _wait(self, s):
        if not self.dry_run:
            time.sleep(s)
