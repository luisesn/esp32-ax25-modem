# Servidor de desarrollo (consola serie, flasheo serie y OTA)

```bash
python3 server/server.py                 # http://0.0.0.0:8000
python3 server/server.py --https         # https://0.0.0.0:8443 (certificado autofirmado, requiere openssl)
python3 server/server.py --port 9000
```

Solo biblioteca estándar de Python. Sirve `build/` según `build/flasher_args.json`
(compila antes con `ninja -C build`).

- **SPIFFS**: Por defecto la pestaña Flasheo ofrece el botón «Flashear solo SPIFFS» y la casilla
  «incluir SPIFFS en todo» (marcada; pide confirmación: sobrescribe `config.json` y la web del dispositivo). `--no-spiffs` lo desactiva.
- **Consola serie y flasheo por serie**: Web Serial + [esptool-js](https://github.com/espressif/esptool-js)
  0.5.7 (incluido en `static/`, Apache-2.0) corren en el **navegador** (Chrome/Edge): el USB del
  dispositivo debe estar en la máquina del navegador. Web Serial exige contexto seguro:
  `http://localhost` (p. ej. reenvío de puertos de VS Code) o `--https`. Por `http://<ip>` no funciona.
- **OTA**: la hace el **navegador**, directamente a `http://<ip>/api/ota/upload` (IP por defecto
  `192.168.4.1`): el dispositivo está en la red de quien ve la web, no en la del servidor, que solo
  sirve el `.bin`. Sin SHA-256 (el firmware no responde a preflight CORS) y la confirmación es que el
  dispositivo se reinicia. No funciona con `--https` (el navegador bloquea la subida a `http://`).
- **Seguridad**: no hay autenticación; úsalo solo en una red de confianza (o reenviando el puerto
  a `localhost`). `spiffs.bin` se sirve por defecto y lleva `config.json` con credenciales WiFi:
  quien alcance el puerto puede descargarlo; usa `--no-spiffs` para impedirlo. Los `*.pem` generados están
  en `.gitignore`.
