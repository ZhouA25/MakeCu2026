"""List serial devices with their stable names and what they probably are.

  python -m pillbot.tools.ports            list + probe for the hub
  python -m pillbot.tools.ports --no-probe just list (doesn't open any port)

Copy the suggested lines into configs/robot.local.yaml. Stable /dev/serial/by-id/... names don't
change when you replug devices or add new ones (Linux / Raspberry Pi). On macOS use the /dev/cu.* name.
"""
import argparse

from pillbot.utils.ports import list_serial, probe_hub


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-probe", action="store_true", help="don't open ports to look for the hub")
    args = ap.parse_args(argv)
    ports = list_serial()
    if not ports:
        print("No USB serial devices found. Plug them in (and use a data cable).")
        return
    hub = None
    for p in ports:
        is_hub = (not args.no_probe) and probe_hub(p["device"])
        if is_hub and hub is None:
            hub = p
        vidpid = f"{p['vid']:04x}:{p['pid']:04x}" if p["vid"] is not None else "-"
        print(f"\n{p['device']}   [{vidpid}]{'   <-- PillBot HUB' if is_hub else ''}")
        print(f"   probably: {p['guess']}")
        if p["by_id"]:
            print(f"   stable:   {p['by_id']}")
    print("\nSuggested configs/robot.local.yaml lines:")
    if hub:
        print("hub:\n  enabled: true\n  port: " + (hub["by_id"] or hub["device"]) + "    # or: auto")
    elif not args.no_probe:
        print("  (no hub found — is the ESP32 flashed with firmware/esp32_hub and the Serial Monitor closed?)")
    others = [p for p in ports if p is not hub]
    for p in others:
        g = p["guess"].lower()
        name = p["by_id"] or p["device"]
        if "u2d2" in g or "dynamixel" in g or "ft232h" in g:
            print(f"follower:\n  port: {name}    # Dynamixel arm adapter")
    if any(p is not hub for p in ports):
        print("# Other devices (SensX adapter, arm adapter...): unplug one at a time to see which is which.")


if __name__ == "__main__":
    main()
