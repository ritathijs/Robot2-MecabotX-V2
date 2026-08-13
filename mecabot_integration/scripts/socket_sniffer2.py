#!/usr/bin/env python3
import os
import time
import logging
import json
import socketio

# 1. Configuratielogger instellen (schrijft naar bestand én terminal)
log_filename = "socket_traffic.log"
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_filename, encoding="utf-8"),
        logging.StreamHandler()  # Toont het live in je terminal
    ]
)

# Server URL van de Raspberry Pi
SERVER_URL = os.environ.get("QUIZ_SERVER_URL", "http://10.0.0.11:80")

# 2. De custom Sniffer Client maken
class SocketSniffer(socketio.Client):
    def __init__(self, *args, **kwargs):
        # We zetten logger=False om de interne, ruwe bibliotheek-logs te onderdrukken
        super().__init__(*args, logger=False, engineio_logger=False, **kwargs)

    def emit(self, event, data=None, namespace=None, callback=None):
        """Onderschep en log álle UITGAANDE berichten."""
        try:
            readable_data = json.dumps(data, indent=2) if data else "Geen data"
        except Exception:
            readable_data = str(data)
            
        logging.info(f"\n[⬆️  UITGAAND] Event: '{event}'\nData: {readable_data}\n" + "-"*50)
        
        # Voer de daadwerkelijke verzending uit
        return super().emit(event, data, namespace, callback)

# Initialiseer de sniffer client
sio = SocketSniffer(reconnection=True, reconnection_attempts=5)

# 3. BINNENKOMEND: Catch-all handler voor alle events
@sio.on('*')
def catch_all_incoming(event, data=None):
    """Onderschep en log álle BINNENKOMENDE berichten van de Pi server."""
    try:
        readable_data = json.dumps(data, indent=2) if data else "Geen data"
    except Exception:
        readable_data = str(data)
        
    logging.info(f"\n[⬇️  BINNENKOMEND] Event: '{event}'\nData: {readable_data}\n" + "-"*50)

# Standaard systeem-callbacks loggen
@sio.event
def connect():
    logging.info(f"🟢 Succesvol verbonden met de Pi server op {SERVER_URL}")
    logging.info("Identificatie 'orin-nano-robot' verzenden...")
    sio.emit("identification", "orin-nano-robot")

@sio.event
def disconnect():
    logging.info("🔴 Verbinding met de Pi server verbroken.")

@sio.event
def connect_error(data):
    logging.error(f"⚠️  Verbindingsfout opgetreden: {data}")

# 4. Main execution loop
if __name__ == "__main__":
    logging.info(f"Sniffer opstarten... Probeert te verbinden met {SERVER_URL}")
    try:
        # Maak verbinding met de server
        sio.connect(SERVER_URL, transports=['websocket'])
        
        # Houd het script in leven om te blijven sniffen
        sio.wait()
    except KeyboardInterrupt:
        logging.info("\n🛑 Sniffer handmatig gestopt via KeyboardInterrupt.")
    except Exception as e:
        logging.error(f"Fout bij starten sniffer: {e}")
    finally:
        if sio.connected:
            sio.disconnect()

