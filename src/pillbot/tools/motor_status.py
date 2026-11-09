"""What each servo is doing: errors, temperature, current, position, limits. Changes nothing
(unless --reboot). Close any other script using the arm first (one program per port).

  python -m pillbot.tools.motor_status                 table for every motor
  python -m pillbot.tools.motor_status --watch         refresh every 0.5 s (move joints by hand / watch heat)
  python -m pillbot.tools.motor_status --reboot gripper   clear a tripped servo (overload etc.) — no unplugging
  python -m pillbot.tools.motor_status --reboot all

A servo that hits an error (most often OVERLOAD: it pushed at its current limit for too long) cuts its
own torque and stays limp — "losing force" — until it's rebooted or power-cycled. Its LED blinks.
"""
import argparse
import sys
import time

from pillbot.control.robot_io import make_robot
from pillbot.utils.config import load_yaml

ERRORS = {0: "input voltage", 2: "OVERHEATING", 3: "motor encoder", 4: "electrical shock", 5: "OVERLOAD"}
MODES = {0: "current", 1: "velocity", 3: "position", 4: "extended", 5: "current+position", 16: "pwm"}
FIELDS = ["Torque_Enable", "Hardware_Error_Status", "Present_Temperature", "Present_Input_Voltage",
          "Present_Current", "Present_Position", "Goal_Position", "Min_Position_Limit", "Max_Position_Limit",
          "Operating_Mode", "Goal_Current", "Current_Limit"]


def decode_errors(v: int) -> str:
    return ", ".join(name for bit, name in ERRORS.items() if v >> bit & 1) or "-"


def read_all(bus) -> dict[str, dict]:
    out = {}
    for motor in bus.motors:
        row = {}
        for f in FIELDS:
            try:
                row[f] = int(bus.read(f, motor, normalize=False))
            except Exception:  # noqa: BLE001 — a register a model doesn't have, or no answer
                row[f] = None
        out[motor] = row
    return out


def table(rows: dict[str, dict]) -> str:
    def g(r, k, fmt="{}"):
        return "?" if r[k] is None else fmt.format(r[k])
    lines = [f"{'motor':14s} {'torque':6s} {'temp':>5s} {'volt':>5s} {'mA':>6s} {'pos':>6s} {'goal':>6s} "
             f"{'limits':>13s} {'mode':16s} errors"]
    for m, r in rows.items():
        cur = "?" if r["Present_Current"] is None else str(r["Present_Current"])   # XL330: ~1 mA per unit
        volt = "?" if r["Present_Input_Voltage"] is None else f"{r['Present_Input_Voltage'] / 10:.1f}"
        limits = f"{g(r, 'Min_Position_Limit')}..{g(r, 'Max_Position_Limit')}"
        mode = MODES.get(r["Operating_Mode"], str(r["Operating_Mode"]))
        if r["Operating_Mode"] == 5 and r["Goal_Current"] is not None:
            mode += f" {r['Goal_Current']}mA"
        err = "?" if r["Hardware_Error_Status"] is None else decode_errors(r["Hardware_Error_Status"])
        lines.append(f"{m:14s} {('on' if r['Torque_Enable'] else 'OFF'):6s} {g(r, 'Present_Temperature', '{}C'):>5s} "
                     f"{volt:>5s} {cur:>6s} {g(r, 'Present_Position'):>6s} {g(r, 'Goal_Position'):>6s} "
                     f"{limits:>13s} {mode:16s} {err}")
    return "\n".join(lines)


def hints(rows: dict[str, dict]) -> list[str]:
    out = []
    for m, r in rows.items():
        e = r["Hardware_Error_Status"] or 0
        if e:
            out.append(f"{m}: {decode_errors(e)} — it has cut its torque. Clear it: "
                       f"python -m pillbot.tools.motor_status --reboot {m}")
        if e >> 5 & 1:
            out.append(f"{m}: overload = it pushed at its limit too long. Lower gripper_goal_current / "
                       "current_limit, or don't squeeze against a hard stop.")
        if (r["Present_Temperature"] or 0) >= 60:
            out.append(f"{m}: {r['Present_Temperature']}C — hot. Let it cool; lower its current.")
        lo, hi = r["Min_Position_Limit"], r["Max_Position_Limit"]
        if m == "gripper" and lo is not None and hi is not None and hi - lo < 200:
            out.append(f"gripper: position limits {lo}..{hi} — too narrow to move. Run: python -m pillbot.tools.gripper_range")
    return out


# --- reboot (Dynamixel Protocol 2.0 instruction 0x08), sent with pyserial ------------------------------
def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def reboot_packet(motor_id: int) -> bytes:
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, motor_id, 0x03, 0x00, 0x08])
    crc = _crc16(body)
    return body + bytes([crc & 0xFF, crc >> 8])


def reboot(port: str, baud: int, ids: list[int]):
    import serial
    with serial.Serial(port, baud, timeout=0.2) as ser:
        for i in ids:
            ser.reset_input_buffer()
            ser.write(reboot_packet(i))
            ser.flush()
            time.sleep(0.05)
            ser.read(64)                                  # status reply, if any
    time.sleep(1.0)                                       # servos restart (~0.5 s), torque comes back OFF


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="store_true", help="refresh every 0.5 s until Ctrl-C")
    ap.add_argument("--reboot", nargs="+", metavar="MOTOR", help="motor names, or 'all'")
    ap.add_argument("--config", default="configs/robot.yaml")
    args = ap.parse_args(argv)

    cfg = load_yaml(args.config)
    robot = make_robot(cfg["follower"])
    bus = robot.bus
    if args.reboot:
        names = list(bus.motors) if args.reboot == ["all"] else args.reboot
        unknown = [n for n in names if n not in bus.motors]
        if unknown:
            sys.exit(f"unknown motor(s) {unknown}; choose from {list(bus.motors)} or all")
        ids = [bus.motors[n].id for n in names]
        print(f"rebooting {names} (ids {ids}) on {cfg['follower']['port']} — they'll come back with torque OFF")
        reboot(cfg["follower"]["port"], int(bus.default_baudrate), ids)
    bus.connect()                                          # plain bus: no calibration prompt, no configure
    try:
        while True:
            rows = read_all(bus)
            if args.watch:
                print("\033[2J\033[H", end="")
            print(table(rows))
            for h in hints(rows):
                print("  -> " + h)
            if not args.watch:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect(disable_torque=False)               # leave each motor as it was
    return 0


if __name__ == "__main__":
    sys.exit(main())
