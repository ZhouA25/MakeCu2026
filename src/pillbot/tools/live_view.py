"""Live viewer — watch the robot's camera from any browser, as many viewers as you like.

  python -m pillbot.tools.live_view               then open  http://<pi-or-laptop>:8080
  python -m pillbot.run --view                    same viewer, during a real run

The camera itself still has ONE client (this process), so nothing else may open its stream at the
same time. The page shows the annotated video (or the detection mask), detection info, run status,
the SensX pressure grid, and a button to (re)capture the empty-zone background.
"""
import argparse
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

log = logging.getLogger(__name__)

PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>PillBot live</title><style>
body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}
header{padding:10px 14px;background:#1d1d1d;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
h1{font-size:17px;margin:0 12px 0 0}button{background:#2d6cdf;color:#fff;border:0;border-radius:6px;padding:7px 12px;cursor:pointer}
button.alt{background:#444}main{display:flex;gap:14px;padding:14px;flex-wrap:wrap}
img{max-width:100%;width:640px;image-rendering:pixelated;border-radius:6px;background:#000}
.card{background:#1d1d1d;border-radius:8px;padding:12px;min-width:220px}td{padding:2px 8px 2px 0}
.k{color:#999}#grid{display:grid;grid-template-columns:repeat(5,34px);gap:3px;margin-top:6px}
#grid div{height:34px;border-radius:4px;background:#222}#msg{color:#ffd166}
</style></head><body>
<header><h1>PillBot live</h1>
<button onclick="setView('annotated')">Video</button><button class="alt" onclick="setView('mask')">Mask</button>
<button class="alt" onclick="setView('raw')">Raw</button>
<button onclick="bg()">Capture background (clear the zone!)</button><span id="msg"></span></header>
<main><img id="v" src="/stream?view=annotated">
<div class="card"><table id="t"></table><div class="k" style="margin-top:10px">SensX pressure</div><div id="grid"></div></div></main>
<script>
function setView(v){document.getElementById('v').src='/stream?view='+v+'&t='+Date.now()}
async function bg(){const r=await fetch('/background',{method:'POST'});document.getElementById('msg').textContent=(await r.json()).msg}
const grid=document.getElementById('grid');for(let i=0;i<25;i++)grid.appendChild(document.createElement('div'));
async function tick(){try{const s=await (await fetch('/status.json')).json();
const rows=[['camera',s.camera],['fps',s.fps],['detector',s.method],['bottle',s.bottle],['zone s,t',s.zone],['width',s.width],
['profile',s.profile],['ignored',s.ignored],['run',s.run],['last log',s.last_log],['camera error',s.error]].filter(r=>r[1]!==null&&r[1]!==''&&r[1]!==undefined);
document.getElementById('t').innerHTML=rows.map(r=>'<tr><td class=k>'+r[0]+'</td><td>'+r[1]+'</td></tr>').join('');
const tg=s.tactile;[...grid.children].forEach((c,i)=>{const v=tg?tg.flat()[i]||0:0,m=tg?Math.max(1,...tg.flat()):1;
c.style.background=tg?`rgb(${Math.round(255*v/m)},${Math.round(80*v/m)},40)`:'#222'})}catch(e){}}
setInterval(tick,400);tick();
</script></body></html>"""


class LiveView:
    def __init__(self, cfg: dict, camera, port: int = 8080, host: str = "0.0.0.0", detector=None, mapper=None):
        from pillbot.perception.detect_bottle import make_detector, settings
        self.cfg, self.camera, self.port, self.host = cfg, camera, port, host
        self.st = settings(cfg)
        if mapper is None:
            try:
                from pillbot.perception.zone import ZoneMapper
                mapper = ZoneMapper.load()
            except FileNotFoundError:
                mapper = None
        self.mapper = mapper
        self.detector = detector if detector is not None else make_detector(cfg, mapper=mapper)
        self.status: dict = {"run": "idle", "last_log": ""}
        self.tactile_fn = None            # optional callable -> 2D array (SensX frame)
        self._server = None

    # ---------------------------------------------------------------- frames
    def render(self, view: str = "annotated"):
        from pillbot.control.bottle_select import match_bottle
        from pillbot.perception.detect_bottle import annotate, width_mm
        frame, _ = self.camera.latest()
        if frame is None:
            img = np.zeros((240, 320, 3), np.uint8)
            cv2.putText(img, "waiting for camera...", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            return img
        if view == "raw":
            return frame
        if view == "mask":
            m = self.detector.mask(frame) if hasattr(self.detector, "mask") else None
            return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR) if m is not None else frame
        det = self.detector(frame)
        anchor = self.st.get("anchor", "base")
        rej = getattr(self.detector, "rejects", ())
        info, upd = "", {"bottle": "not found", "zone": None, "width": None, "profile": None,
                         "ignored": ", ".join(r for _, r in rej) or None}
        if det:
            upd["bottle"] = f"{det.w}x{det.h}px"
            if self.mapper:
                s, t = self.mapper.to_zone(*det.anchor_px(anchor))
                upd["zone"] = f"{s:.2f}, {t:.2f}" + ("" if self.mapper.inside(s, t) else " (outside)")
                w = width_mm(det, self.mapper, self.st["zone_size_mm"], anchor)
                if w:
                    b = match_bottle(w)
                    upd["width"], upd["profile"] = f"{w:.0f} mm", (b["name"] if b else "no match")
                    info = f"~{w:.0f} mm" + (f" -> {b['name']}" if b else "")
        self.status.update(upd)
        return annotate(frame, det, self.mapper, None, info, anchor, getattr(self.detector, "rejects", ()))

    def capture_background(self, n: int = 12) -> str:
        if not hasattr(self.detector, "capture"):
            return f"detector '{self.detector.name}' doesn't use a background"
        frames, after = [], time.time()
        for _ in range(n):
            f = self.camera.wait_new(after, timeout=2)
            if f is None:
                break
            frames.append(f)
            after = self.camera.latest()[1]
        if not frames:
            return "no frames from camera"
        self.detector.capture(frames)
        self.detector.save()
        return f"background captured ({len(frames)} frames)"

    def status_json(self) -> dict:
        s = dict(self.status)
        s.update(camera=self.camera.name, fps=f"{self.camera.fps:.0f}", error=self.camera.error,
                 method=self.detector.name)
        if self.tactile_fn:
            try:
                f = self.tactile_fn()
                s["tactile"] = None if f is None else np.asarray(f, float).round(0).tolist()
            except Exception:
                s["tactile"] = None
        return s

    # ---------------------------------------------------------------- http
    def _handler(self):
        view = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, ctype, body: bytes):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                if u.path == "/":
                    return self._send(200, "text/html; charset=utf-8", PAGE.encode())
                if u.path == "/status.json":
                    return self._send(200, "application/json", json.dumps(view.status_json()).encode())
                if u.path == "/snapshot.jpg":
                    ok, jpg = cv2.imencode(".jpg", view.render(q.get("view", ["annotated"])[0]))
                    return self._send(200, "image/jpeg", jpg.tobytes())
                if u.path == "/stream":
                    mode = q.get("view", ["annotated"])[0]
                    frames = int(q.get("frames", ["0"])[0])        # 0 = forever (tests use a few)
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    sent, last = 0, 0.0
                    try:
                        while frames == 0 or sent < frames:
                            f = view.camera.wait_new(last, timeout=1.0)
                            _, last = view.camera.latest()
                            ok, jpg = cv2.imencode(".jpg", view.render(mode), [cv2.IMWRITE_JPEG_QUALITY, 75])
                            b = jpg.tobytes()
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(b)).encode() + b"\r\n\r\n" + b + b"\r\n")
                            sent += 1
                            if f is None:
                                time.sleep(0.2)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return
                self._send(404, "text/plain", b"not found")

            def do_POST(self):
                if urlparse(self.path).path == "/background":
                    msg = view.capture_background()
                    return self._send(200, "application/json", json.dumps({"msg": msg}).encode())
                self._send(404, "text/plain", b"not found")

        return H

    def start(self):
        self._server = ThreadingHTTPServer((self.host, self.port), self._handler())
        self._server.daemon_threads = True
        self.port = self._server.server_address[1]
        threading.Thread(target=self._server.serve_forever, daemon=True, name="live-view").start()
        log.info("live view: http://%s:%d", "localhost" if self.host == "0.0.0.0" else self.host, self.port)
        return self

    def stop(self):
        if self._server:
            self._server.shutdown()
            self._server.server_close()


class StatusLogHandler(logging.Handler):
    """Mirror pillbot log lines into the viewer's status panel."""

    def __init__(self, view: LiveView):
        super().__init__(logging.INFO)
        self.view = view

    def emit(self, record):
        msg = record.getMessage()
        self.view.status["last_log"] = msg[:120]
        for key, phase in (("bottle at zone", "picking"), ("pick:", "picking"), ("open:", "opening"),
                           ("opened!", "done ✔"), ("failed", "failed ✖"), ("STOP", "STOPPED")):
            if key in msg:
                self.view.status["run"] = phase


def start_live_view(cfg: dict, port: int = 8080, tactile=None) -> LiveView:
    from pillbot.perception.shared_camera import SharedCamera
    view = LiveView(cfg, SharedCamera.get(cfg), port=port)
    if tactile is not None and getattr(tactile, "available", False):
        view.tactile_fn = tactile.read
    view.start()
    logging.getLogger("pillbot").addHandler(StatusLogHandler(view))
    return view


def main(argv=None):
    import socket
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--camera", help="camera name from robot.yaml (default: perception_camera)")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    from pillbot.perception.shared_camera import SharedCamera
    from pillbot.utils.config import load_yaml
    cfg = load_yaml(args.config)
    view = LiveView(cfg, SharedCamera.get(cfg, args.camera), port=args.port).start()
    host = socket.gethostname()
    print(f"\nOpen  http://{host}.local:{view.port}  (or http://<this machine's IP>:{view.port})   Ctrl-C to stop\n")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        view.stop()


if __name__ == "__main__":
    main()
