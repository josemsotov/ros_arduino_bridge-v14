#!/usr/bin/env python3
"""
voice_control.py — Control por voz para Smart Golf Trolley
Escucha continuamente el micrófono (audífonos BT o USB) y convierte
comandos de voz en llamadas a la API del robot.

Dependencias: pip3 install vosk sounddevice requests
Modelos:      https://alphacephei.com/vosk/models → vosk-model-small-es-0.42

Uso: python3 voice_control.py [--list-devices] [--device N]
"""
import argparse, json, logging, os, queue, sys, threading
import urllib.request, urllib.error

API = "http://127.0.0.1:8080"
MODEL_DIR = os.path.expanduser("~/vosk-model-es")
LOG = logging.getLogger("voice_ctrl")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

# ── Mapa voz → acción ────────────────────────────────────────────────────────
COMMANDS = {
    "adelante":   ("POST", "/api/follower", {"command": "FOLLOW"}),
    "seguir":     ("POST", "/api/follower", {"command": "FOLLOW"}),
    "para":       ("POST", "/api/stop",     {}),
    "parar":      ("POST", "/api/stop",     {}),
    "stop":       ("POST", "/api/stop",     {}),
    "stadia":     ("POST", "/api/stadia/start", {}),
    "mando":      ("POST", "/api/stadia/start", {}),
    "stadia off": ("POST", "/api/stadia/stop",  {}),
    "mando off":  ("POST", "/api/stadia/stop",  {}),
    "golf":       None,  # cambia de panel – no tiene API directa
    "calibrar":   None,
}

def api_call(method, path, body=None):
    url = API + path
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(url, data=data, method=method,
                                  headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            LOG.info("API %s %s → %s", method, path, r.status)
    except urllib.error.URLError as e:
        LOG.warning("API error: %s", e)

def process_text(text: str):
    text = text.lower().strip()
    LOG.info("Recognized: %r", text)
    # check multi-word first
    for phrase, action in sorted(COMMANDS.items(), key=lambda x: -len(x[0])):
        if phrase in text and action:
            api_call(*action)
            return

def listen(device=None):
    try:
        import sounddevice as sd
        import vosk
    except ImportError:
        LOG.error("Instala dependencias: pip3 install vosk sounddevice")
        LOG.error("Descarga modelo español: https://alphacephei.com/vosk/models")
        LOG.error("Extrae en: %s", MODEL_DIR)
        sys.exit(1)

    if not os.path.isdir(MODEL_DIR):
        LOG.error("Modelo no encontrado en %s", MODEL_DIR)
        LOG.error("Descarga vosk-model-small-es-0.42 y extráelo como %s", MODEL_DIR)
        sys.exit(1)

    model = vosk.Model(MODEL_DIR)
    q: queue.Queue = queue.Queue()
    samplerate = 16000

    def callback(indata, frames, time_info, status):
        q.put(bytes(indata))

    rec = vosk.KaldiRecognizer(model, samplerate)
    LOG.info("Escuchando (Ctrl+C para salir)...")
    with sd.RawInputStream(samplerate=samplerate, blocksize=8000,
                            device=device, dtype='int16', channels=1,
                            callback=callback):
        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get("text", "").strip()
                if text:
                    process_text(text)

# ── HTTP server para arranque/parada desde la UI ────────────────────────────
_voice_thread: threading.Thread | None = None
_stop_event = threading.Event()

def start_server(port=8765):
    from http.server import BaseHTTPRequestHandler, HTTPServer
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            global _voice_thread
            if self.path == '/start' and (_voice_thread is None or not _voice_thread.is_alive()):
                _stop_event.clear()
                _voice_thread = threading.Thread(target=listen, daemon=True)
                _voice_thread.start()
                self.send_response(200); self.end_headers()
                self.wfile.write(b'{"ok":true,"voice":"started"}')
            elif self.path == '/stop':
                _stop_event.set()
                self.send_response(200); self.end_headers()
                self.wfile.write(b'{"ok":true,"voice":"stopped"}')
            else:
                self.send_response(400); self.end_headers()
        def log_message(self, *a): pass
    server = HTTPServer(('127.0.0.1', port), Handler)
    LOG.info("Voice control server on port %d", port)
    server.serve_forever()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--list-devices', action='store_true')
    parser.add_argument('--device', type=int, default=None)
    parser.add_argument('--server', action='store_true',
                        help='Run HTTP control server (for UI integration)')
    args = parser.parse_args()

    if args.list_devices:
        import sounddevice as sd
        print(sd.query_devices())
        sys.exit(0)

    if args.server:
        start_server()
    else:
        listen(device=args.device)
