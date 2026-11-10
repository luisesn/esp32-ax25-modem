#!/usr/bin/env python3
"""Servidor de desarrollo para el módem APRS (HTTP por defecto, HTTPS con --https).

- Sirve una interfaz web (consola serie + flasheo por serie + OTA desde el
  navegador) y los binarios de ../build (según build/flasher_args.json).
- La consola serie y el flasheo por serie usan Web Serial + esptool-js en el
  NAVEGADOR: el puerto USB debe estar en la máquina donde corre el navegador
  (Chrome/Edge). Web Serial exige contexto seguro: sirve http://localhost
  (p. ej. con el reenvío de puertos de VS Code) o https (opción --https).
  Con http por IP de red, la consola serie y el flasheo serie NO funcionan.
- La OTA la hace el NAVEGADOR (el dispositivo está en la red de quien ve la
  web, no en la del servidor): este servidor solo sirve el .bin. Con --https
  el navegador bloquea la subida a http://<dispositivo> (contenido mixto).

Solo biblioteca estándar. Con --https requiere el binario `openssl` para
generar el certificado autofirmado la primera vez (o pasar --cert/--key).
"""
import argparse
import hashlib
import ipaddress
import json
import os
import socket
import ssl
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
STATIC = {  # lista blanca de ficheros estáticos (sin recorrido de rutas)
    "/": ("index.html", "text/html; charset=utf-8"),
    "/esptool-js.js": ("esptool-js.js", "text/javascript; charset=utf-8"),
    "/ESPTOOL-JS-LICENSE": ("ESPTOOL-JS-LICENSE", "text/plain; charset=utf-8"),
}


class Config:
    build_dir: Path
    allow_spiffs: bool


CFG = Config()

# ── Firmware (build/flasher_args.json) ───────────────────────────────────────


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def load_flash_files():
    """Devuelve (lista de ficheros {offset, path, size, role}, flash_settings)."""
    fa = CFG.build_dir / "flasher_args.json"
    if not fa.is_file():
        return [], {}
    data = json.loads(fa.read_text())
    roles = {}
    for role in ("bootloader", "partition-table", "otadata", "spiffs", "app"):
        if role in data:
            roles[data[role]["file"]] = role
    files = []
    for off, rel in data.get("flash_files", {}).items():
        role = roles.get(rel, "other")
        if role == "spiffs" and not CFG.allow_spiffs:
            continue  # contiene config.json con credenciales WiFi
        p = (CFG.build_dir / rel).resolve()
        try:
            p.relative_to(CFG.build_dir.resolve())
        except ValueError:
            continue
        if p.is_file():
            files.append({"offset": int(off, 16), "path": rel, "size": p.stat().st_size, "role": role})
    files.sort(key=lambda f: f["offset"])
    return files, data.get("flash_settings", {})


def app_file():
    files, _ = load_flash_files()
    for f in files:
        if f["role"] == "app":
            return CFG.build_dir / f["path"]
    return None


# ── HTTP ─────────────────────────────────────────────────────────────────────


class Handler(BaseHTTPRequestHandler):
    server_version = "aprs-dev-server"

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    # util
    def _send(self, code, body, ctype="application/json", extra=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in STATIC:
            name, ctype = STATIC[path]
            return self._send(200, (HERE / "static" / name).read_bytes(), ctype)
        if path == "/ui/index.html":   # web del dispositivo (para subirla por SPIFFS sin cable)
            f = HERE.parent / "main" / "spiffs_data" / "index.html"
            if not f.is_file():
                return self._send(404, {"error": "no existe main/spiffs_data/index.html"})
            return self._send(200, f.read_bytes(), "text/html; charset=utf-8")
        if path == "/api/fw":
            files, settings = load_flash_files()
            app = app_file()
            info = None
            if app:
                st = app.stat()
                info = {"size": st.st_size, "mtime": int(st.st_mtime), "sha256": sha256_file(app)}
            version = None
            try:
                version = json.loads((CFG.build_dir / "project_description.json").read_text()).get("project_version")
            except (OSError, ValueError):
                pass
            return self._send(200, {"files": files, "flash_settings": settings, "app": info, "version": version, "spiffs_allowed": CFG.allow_spiffs,
                                    })
        if path.startswith("/fw/"):
            rel = path[len("/fw/"):]
            files, _ = load_flash_files()
            if rel not in {f["path"] for f in files}:
                return self._send(404, {"error": "no listado"})
            data = (CFG.build_dir / rel).read_bytes()
            return self._send(200, data, "application/octet-stream")
        self._send(404, {"error": "no encontrado"})

# ── Certificado ──────────────────────────────────────────────────────────────


def local_ips():
    ips = {"127.0.0.1"}
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))  # no envía nada; solo elige interfaz
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips)


def ensure_cert(cert: Path, key: Path, extra_san):
    if cert.is_file() and key.is_file():
        return
    sans = ["DNS:localhost", f"DNS:{socket.gethostname()}"]
    sans += [f"IP:{ip}" for ip in local_ips()]
    for s in extra_san:
        try:
            ipaddress.ip_address(s)
            sans.append(f"IP:{s}")
        except ValueError:
            sans.append(f"DNS:{s}")
    print(f"Generando certificado autofirmado ({', '.join(sans)})…")
    subprocess.run(
        ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "825",
         "-keyout", str(key), "-out", str(cert), "-subj", "/CN=esp32-aprs-modem-dev",
         "-addext", "subjectAltName=" + ",".join(sans)],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    os.chmod(key, 0o600)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=None,
                    help="puerto (por defecto 8000 en http, 8443 con --https)")
    ap.add_argument("--https", action="store_true",
                    help="sirve por HTTPS con certificado autofirmado (o --cert/--key)")
    ap.add_argument("--build-dir", default=str(HERE.parent / "build"))
    ap.add_argument("--cert", default=str(HERE / "cert.pem"))
    ap.add_argument("--key", default=str(HERE / "key.pem"))
    ap.add_argument("--san", action="append", default=[], help="IP/DNS extra para el certificado (repetible)")
    ap.add_argument("--no-spiffs", action="store_true",
                    help="no sirve ni permite flashear spiffs.bin (contiene config.json con credenciales WiFi)")
    a = ap.parse_args()

    CFG.build_dir = Path(a.build_dir).resolve()
    CFG.allow_spiffs = not a.no_spiffs

    port = a.port or (8443 if a.https else 8000)
    srv = ThreadingHTTPServer((a.host, port), Handler)
    if a.https:
        cert, key = Path(a.cert), Path(a.key)
        ensure_cert(cert, key, a.san)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        ctx.load_cert_chain(str(cert), str(key))
        srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
    scheme = "https" if a.https else "http"
    print(f"Build dir : {CFG.build_dir}")
    app = app_file()
    print(f"Binario   : {app if app else 'NO ENCONTRADO (compila primero)'}")
    for ip in (local_ips() if a.host == "0.0.0.0" else [a.host]):
        print(f"URL       : {scheme}://{ip}:{port}/")
    if a.https:
        print("(certificado autofirmado: el navegador pedirá aceptarlo)")
    else:
        print("(HTTP: Web Serial solo funciona en http://localhost; para otra IP usa --https)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
