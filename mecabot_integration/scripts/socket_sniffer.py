#!/usr/bin/env python3
import socketio
import json
from datetime import datetime

LOGFILE = "socket.log"

# ---- KLEUREN ----
RESET = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
RED = "\033[91m"

# ---- LOGFUNCTIE ----
def log(direction, event, data):
    timestamp = datetime.now().isoformat()
    line = f"[{timestamp}] {direction} {event}: {json.dumps(data)}\n"

    with open(LOGFILE, "a") as f:
        f.write(line)

    print(
        f"{CYAN}[{timestamp}]{RESET} "
        f"{direction} {YELLOW}{event}{RESET} "
        f"{MAGENTA}{json.dumps(data)}{RESET}"
    )

# ---- SOCKET.IO CLIENT ----
sio = socketio.Client(logger=False, engineio_logger=False)

SERVER = "http://10.0.0.11"   # PAS AAN naar jouw PI-poort

# ---- CONNECT ----
@sio.event
def connect():
    log(GREEN + "⬅ CONNECT" + RESET, "connect", {})

@sio.event
def disconnect():
    log(RED + "⬅ DISCONNECT" + RESET, "disconnect", {})

# ---- INKOMENDE EVENTS ----
@sio.on("*")
def catch_all(event, data):
    log(GREEN + "⬅ IN" + RESET, event, data)

# ---- UITGAANDE EVENTS ----
_original_emit = sio.emit

def emit_with_log(event, data=None, *args, **kwargs):
    log(YELLOW + "➡ OUT" + RESET, event, data)
    return _original_emit(event, data, *args, **kwargs)

sio.emit = emit_with_log

# ---- START ----
print(GREEN + "🔍 Python Socket Sniffer gestart…" + RESET)
print(f"Verbinden met {SERVER} ...")

try:
    sio.connect(SERVER, transports=["websocket"])
    sio.wait()
except Exception as e:
    print(RED + f"Fout bij verbinden: {e}" + RESET)
