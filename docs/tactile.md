# Tactile sensing (SensX25): how it works, how to use it, how to set it up on the Pi

The cap-gripping finger has a **TouchTronix SensX25**: a 5×5 grid of pressure sensors ("taxels").
The gripper code uses it to **feel when the jaws touch the cap** (stop closing, then squeeze a set
amount more) and to **feel the cap slipping** while the wrist twists.

## Quick reference

```bash
python -m pillbot.tools.tactile_live doctor             # which port is it? is it streaming? what's wrong?
python -m pillbot.tools.tactile_live                    # live numbers = exactly what the gripper code sees
python -m pillbot.tools.tactile_live plot               # heatmap window
python -m pillbot.sensors.tactile_server                # share the sensor with several programs
python -m pillbot.tools.fake_sensx                      # no hardware? a virtual sensor to test against
pytest tests/test_tactile_sensx.py                      # the whole stack against the virtual sensor
```

```python
from pillbot.sensors.tactile import open_tactile

with open_tactile() as tac:          # reads configs/robot.yaml -> tactile:, connects, tares
    s = tac.snapshot()
    s.contact      # True/False: is something touching?
    s.pressure     # total pressure above the tare (sum of 25 taxels, kPa)
    s.centroid     # (row, col) where the pressure is centred; moves if the cap slides
    s.frame        # the raw 5x5 numpy array, or None if the sensor stopped sending
```

---

## 1. The hardware chain

```
 finger pad          sensor board            Waveshare FT232R          Pi / laptop
┌──────────┐  analog  ┌──────────────┐  UART   ┌──────────────┐  USB   ┌──────────────────────────────┐
│ 5×5 grid │ ───────► │ microcontrol-│ ──────► │ USB-to-TTL   │ ─────► │ OS driver → /dev/ttyUSB0      │
│ of taxels│          │ ler + 12-bit │ TX→RX   │ bridge chip  │        │ pyserial → sensx driver       │
└──────────┘          │ ADC          │ RX←TX   └──────────────┘        │ → SensXTactile → grasp.py     │
                      └──────────────┘  GND─GND                        └──────────────────────────────┘
```

**Taxels.** Each of the 25 cells is a pressure-sensitive element: press harder and its electrical
resistance changes. The sensor's own microcontroller measures each cell with a 12-bit
**ADC** (analog-to-digital converter, 0..4095) and converts the reading to **kPa**. By default ("pressure
mode") each taxel reads **0 when untouched, up to 534 kPa**. (For scale: a firm fingertip press is
roughly tens to a couple hundred kPa on the taxels under it.)

**UART** is the simplest way two chips talk: one wire each way plus a shared ground.
- **TX** (transmit) of one side goes to **RX** (receive) of the other, which is why the wires *cross*.
  Swapping them does no harm; you just get silence.
- **GND must be shared**, because a "1" or "0" is a voltage *relative to ground*.
- There's no clock wire, so both sides agree on a speed in advance: the **baud rate**, here **921600 bits/s**.
  Each byte goes out as 10 bits (start bit + 8 data bits + stop bit, "8N1"), so the link carries at
  most 92,160 bytes/s.
- **Logic level:** a "1" is 3.3 V or 5 V depending on the chip. The Waveshare board has a jumper for
  this. Match it to the sensor. Too high can damage a 3.3 V chip; too low may not register at all.

**The FT232R** (the Waveshare board) is a translator: UART on one side, USB on the other. Your computer
sees a USB device and its driver (built into macOS and Linux for FTDI chips) creates a serial port:
`/dev/cu.usbserial-BG00ZVUK` on the Mac, `/dev/ttyUSB0` on the Pi. `BG00ZVUK` is that adapter's
serial number, so the name stays the same for this specific adapter.

> **Only one program can have a serial port open at a time.** That's why `tactile_server` exists (§4).

## 2. The bytes on the wire

**Starting it.** On power-up the sensor waits silently. To start it streaming, the driver sends 8 bytes,
a **Modbus RTU** command (a common industrial protocol):

```
01        device address 1
06        function "write one register"
60 10     register 0x6010 ("streaming")
00 01     value 1 (= on)
57 CF     CRC-16 checksum of the 6 bytes before it
```

After that it streams continuously until it loses power. That's why the doctor reports a silent port as
"maybe a SensX waiting for its start command". The command is only sent to a port you name
(`--port`), never during auto-detect, because another Modbus device on the robot could misread it.

**Each frame is 53 bytes**, about **290 frames per second** (≈15 KB/s, 17% of what the link can carry):

```
FF FF | 00 00  00 0C  01 F4  00 00 … (25 values) | 59
header  taxel0 taxel1 taxel2  (the rest 0)         CRC-8
        = 0    = 12   = 500 kPa
```

- **Header `FF FF`** marks where a frame starts. The stream has no gaps or pauses, so the reader has to
  find frames by searching for this pattern.
- **25 values, 2 bytes each, big-endian** (high byte first): `01 F4` = 1×256 + 244 = **500**. Row 0
  first, 5 per row. Values are 12-bit, so the high byte is never above `0F`, and an `FF FF` can't
  appear inside the data by accident. That's what makes the header search reliable.
- **CRC-8** (a checksum): the sensor computes one byte from the 50 data bytes, and the receiver
  recomputes it. If they differ, a bit was corrupted on the wire (e.g. noise from the servo motors), so
  the driver **throws that frame away**. You lose one frame out of 290 per second; you never act on a
  wrong one.

(The TouchTronix *hub*, for up to 5 sensors, uses headers `AA AA`…`EE EE` and no CRC. We don't use it,
but `sensx_probe.classify` recognizes both, so the doctor can tell you which one you have.)

## 3. The software layers

| Layer | File | Job |
|---|---|---|
| Driver | `sensx_python/sensx/sensor.py` (TouchTronix, git submodule) | opens the port, sends the start command, **background thread** reads bytes, finds `FF FF`, checks the CRC, keeps the **latest frame** |
| Backend | `src/pillbot/sensors/tactile.py` → `SensXTactile` | wraps the driver: tare, pressure, contact, slip, centroid, **stale-data guard** |
| Gripper logic | `control/grasp.py`, `control/open_sequence.py` | close until `in_contact()`, squeeze, twist, check `is_slipping()` |
| Tools | `tools/tactile_live.py`, `tools/fake_sensx.py`, `sensors/tactile_server.py`, `sensors/sensx_probe.py` | find / watch / tune / share / simulate |

**Why a background thread?** Frames arrive every ~3.5 ms whether or not anyone is reading. The thread
keeps the serial buffer drained and always holds the newest frame, so when `grasp.py` asks "touching?"
it gets the current answer immediately, not a stale frame from a backlog. This is called the
**latest-value pattern**: we care about *now*, not about every frame.

**What the numbers mean** (all in `TactileSensor`, the base class every backend shares):

- **Tare** (`tare()`, called by `run.py` at the start of every cycle with the jaws open and empty):
  averages a few frames to get the *baseline*, i.e. whatever the pad reads when nothing touches it
  (e.g. preload from how it's glued on). Same idea as zeroing a kitchen scale.
- **Pressure** = (sum of all 25 taxels) − baseline. A stand-in for grip force: more squeeze, bigger
  number. It's a sum of kPa, **not newtons**. To get newtons you'd calibrate against a scale.
- **Contact** = pressure ≥ `contact_threshold` (configs/robot.yaml). `grip_until_contact` closes the
  gripper in small steps until this turns true, then squeezes `gripper.squeeze` more.
- **Slip** (`is_slipping(grip_pressure)`): after twisting, if pressure dropped below `slip_ratio` ×
  the pressure right after gripping, the cap has slid out of the jaws. `open_sequence` then retries with
  a harder squeeze.
- **Centroid** = pressure-weighted average position: Σ(position × pressure) / Σ pressure. A centroid
  that *moves* while the jaws are still means the cap is sliding across the pad. The code doesn't use
  this yet; it would make an earlier slip warning.
- **Stale guard**: if no new frame has arrived for `max_age_s` (0.25 s), `read()` returns `None`. That
  makes contact = false and pressure = 0, and it logs an error. Without this, an unplugged cable would
  freeze the last frame and the gripper would believe it was still holding the cap.

## 4. Using it from other programs

**a) Inside the pipeline:** already wired up. `run.py` and `open_sequence.py` call
`make_tactile(cfg)`, which picks the backend from `tactile.backend` in the config. `--dry-run` uses a
simulated sensor.

**b) Your own Python script** (e.g. testing gripper servo code on its own):

```python
from pillbot.sensors.tactile import open_tactile

with open_tactile() as tac:                       # or open_tactile(port="/dev/cu.usbserial-BG00ZVUK")
    while not tac.in_contact():
        close_gripper_a_bit()                     # your servo code
    grip = tac.pressure()
    twist()
    if tac.is_slipping(grip):
        squeeze_harder()
```

Use `tac.snapshot()` when you need several values that agree with each other. Separate calls each read
the newest frame, and a new frame arrives every ~3.5 ms.

**c) Several programs at once, or a non-Python program, or another machine:** run the server. It
owns the port, and everyone else connects to it:

```bash
python -m pillbot.sensors.tactile_server                        # this machine only
python -m pillbot.sensors.tactile_server --listen 0.0.0.0:8765  # also other machines (e.g. Pi B)
nc localhost 8765                                               # see the stream
```

- **Python programs:** set `tactile: {backend: net, url: localhost:8765}` in `configs/robot.local.yaml`.
  `make_tactile` then returns a `NetTactile`, and grasp.py etc. work unchanged.
- **Anything else** (C++, an ESP32 over Wi-Fi, a web dashboard): read lines over TCP. Each line is JSON:
  `{"t", "frame", "pressure", "contact", "peak", "centroid"}`, 100 per second.
- Sending through the server adds well under a millisecond on the same machine.

## 5. Tuning `contact_threshold`

1. `python -m pillbot.tools.tactile_live`, with nothing touching the pad (it tares at start).
2. Touch the pad as lightly as the jaws will first touch the cap. Note `pressure`.
3. Squeeze like a real grip. Note `max pressure so far`.
4. Set `contact_threshold` (in `configs/robot.local.yaml`) **above the idle noise but clearly below
   step 2**. The bar shows `|` at the current threshold. 300 is a starting guess, not a measured value.

## 6. Raspberry Pi 5

### USB-TTL adapter or the Pi's GPIO pins?

**Keep the FT232R USB adapter.** The GPIO UART can work, but it gains you nothing here and adds risks:

| | FT232R over USB (recommended) | Pi GPIO 14/15 UART |
|---|---|---|
| Works today | yes, verified on the Mac | untested |
| Same code & config as the laptop | yes, only the port name differs | needs Pi-only config |
| Pi setup | none beyond `dialout` | edit `config.txt`, keep the serial console off those pins, reboot |
| Voltage risk | the adapter's jumper; the Pi is protected | Pi pins are **3.3 V only, not 5 V tolerant**: a 5 V sensor TX can kill the GPIO |
| Long cable down the arm next to servo wires | USB is differential (noise-resistant); put the adapter near the wrist and run USB down the arm | 921600-baud single-ended wire the whole way: more CRC drops |
| Stable device name | `/dev/serial/by-id/usb-FTDI_…_BG00ZVUK…` | `/dev/ttyAMA0` |
| Costs | one USB port (Pi 5 has 4: arm bus board, ESP32 hub, SensX, 1 spare) | nothing |

Switch to GPIO only if you run out of USB ports or need to save the adapter's weight on the wrist.

### Setup with the USB adapter

```bash
# 1. Python 3.12+ (LeRobot needs it). Pi OS Bookworm ships 3.11, so use Miniforge (README §1).
#    The tactile code alone runs on 3.11 (pip install -e sensx_python pyyaml).
git clone --recurse-submodules https://github.com/ZhouA25/MakeCu2026.git && cd MakeCu2026
pip install -e ".[dev]" && pip install -e sensx_python

# 2. Allow your user to open serial ports (once), then log out and back in
sudo usermod -aG dialout $USER

# 3. Plug in the adapter and check
ls /dev/serial/by-id/                    # expect usb-FTDI_FT232R_USB_UART_BG00ZVUK-if00-port0
python -m pillbot.tools.tactile_live doctor --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00ZVUK-if00-port0
python -m pillbot.tools.tactile_live     # robot.yaml already points at that by-id path
```

- Use the **`/dev/serial/by-id/...` path**, not `/dev/ttyUSB0`. `ttyUSB0`/`ttyUSB1` are handed out in
  plug-in order, so the sensor and the ESP32 hub can swap names after a reboot. The by-id name includes
  the adapter's serial number and never changes.
- **Power:** use the official 5 V / 5 A supply. With a weaker one, the Pi 5 limits all USB ports to
  600 mA total, and the arm bus board, hub and sensor share that.
- Headless Pi: `plot` needs a screen. Use the terminal `live` view, or run `tactile_server --listen
  0.0.0.0:8765` on the Pi and connect from a laptop with `backend: net, url: <pi-ip>:8765`.

### If you do go GPIO

1. Check the sensor's logic voltage. It must be **3.3 V**; otherwise add a level shifter.
2. Wire sensor TX → **pin 10** (GPIO15, RXD), sensor RX → **pin 8** (GPIO14, TXD), GND → **pin 6**,
   and VCC → pin 1 (3.3 V) or pin 2 (5 V), whichever the sensor needs.
3. Add `dtparam=uart0=on` to `/boot/firmware/config.txt`. In `sudo raspi-config` → Interface Options →
   Serial Port, answer *login shell: No*, *hardware: Yes*. Reboot.
4. Use `port: /dev/ttyAMA0`. Run the doctor with `--port /dev/ttyAMA0`.

## 7. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Adapter LED on, but no port appears | Charge-only USB cable, or a hub port not passing data. Swap the cable / plug in directly |
| Doctor: "silent even after the start command" | Sensor unpowered, TX/RX swapped, GND not shared, wrong 3.3/5 V jumper, or a loose connector |
| Doctor: "bytes but no SensX frames" | Wrong baud, or noise/bad ground corrupting every frame |
| `permission denied` (Pi) | `sudo usermod -aG dialout $USER`, then log out/in |
| "could not open port … busy" | Another program has it (a live view, Arduino serial monitor, `screen`). Close it, or use `tactile_server` |
| Log: "no new frame for … s" | Sensor stopped mid-run: the cable pulled loose at the wrist? Check the service loop |
| Contact never triggers / always on | Re-tune `contact_threshold` (§5). Always on = it tared while something was touching |
