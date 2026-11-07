"""PillBot Hub — Arduino UNO Q, Linux side.

Relays the MCU (via Bridge RPC) to a TCP socket that speaks the SAME line protocol as the ESP32 hub,
so the laptop/Pi just sets  hub.port: socket://<uno-q-address>:5555  and nothing else changes.

  UNO Q -> client  every 20 ms:  S <ms> <clip_mm> <zone_mm> <buttons>     (on connect: READY pillbot-hub-unoq 1)
  client -> UNO Q:               SV <ch> <us> | RL <0|1> | PING
Safety: when the client disconnects (laptop crash, cable pulled) the relay is switched off.
"""
import socket
import threading
import time

from arduino.app_utils import App, Bridge

PORT = 5555
_bridge_lock = threading.Lock()
_clients: list[socket.socket] = []
_clients_lock = threading.Lock()


def bcall(name, *args):
    with _bridge_lock:                      # one RPC at a time
        return Bridge.call(name, *args)


def handle(conn: socket.socket, addr):
    print(f"client connected: {addr}")
    conn.sendall(b"READY pillbot-hub-unoq 1\n")
    with _clients_lock:
        _clients.append(conn)
    try:
        for raw in conn.makefile("r"):
            line = raw.strip()
            try:
                if line == "PING":
                    conn.sendall(b"PONG\n")
                elif line.startswith("SV "):
                    _, ch, us = line.split()
                    bcall("sv", int(ch), int(us))
                elif line.startswith("RL "):
                    bcall("rl", int(line.split()[1]))
                elif line:
                    conn.sendall(f"ERR {line}\n".encode())
            except Exception as e:                       # bad line / RPC error: report, keep going
                conn.sendall(f"ERR {e}\n".encode())
    except OSError:
        pass
    finally:
        with _clients_lock:
            if conn in _clients:
                _clients.remove(conn)
        try:
            bcall("rl", 0)                               # fail safe: cut hobby-servo power
        except Exception:
            pass
        conn.close()
        print(f"client disconnected: {addr}")


def server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(2)
    print(f"PillBot hub listening on :{PORT}")
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=handle, args=(conn, addr), daemon=True).start()


def loop():
    try:
        line = f"S {bcall('state')}\n".encode()
    except Exception as e:
        print(f"state RPC failed: {e}")
        time.sleep(0.2)
        return
    with _clients_lock:
        for c in list(_clients):
            try:
                c.sendall(line)
            except OSError:
                _clients.remove(c)
    time.sleep(0.02)


threading.Thread(target=server, daemon=True).start()
App.run(user_loop=loop)
