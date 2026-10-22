#!/usr/bin/env python3
# AVTECH AGV - Manuel Kontrol Sunucusu (Jetson Orin Nano)
# GUI (Socket.IO, joystick + WASD) → Jetson → Teensy 4.1 (USB seri, JSON satirlari)
#
# Donanim (mevcut mimari): Hoverboard FOC surus motorlari + ICM-20948 IMU + DM556 forklift step motoru.
# Otonom surus, kumanda, kamera YOK — arac sadece GUI'den surulur.
#
# Surus zinciri:
#   GUI  --drive {x,y} (20 Hz, sadece surerken)-->  Jetson  --manual {speed,steer} (sabit 20 Hz)-->  Teensy
#   - Jetson kendi zamanlayicisiyla gonderir: Wi-Fi gecikme dalgalanmasi Teensy'ye yansimaz.
#   - GUI_DEADMAN_S boyunca GUI'den paket gelmezse (sekme kapandi, Wi-Fi koptu) hedef 0 → arac rampa ile durur.
#   - Jetson olurse Teensy MANUAL_CMD_TIMEOUT_MS (500 ms) ve kalp atisi watchdog'u ile durur.
#   - Hiz modu (YAVAS/ORTA/HIZLI) sunucuda tutulur; GUI sadece -1..1 eksen degeri gonderir.
#
# Calistirma:  python3 avtech_brain.py      (GUI: http://<jetson-ip>:5000)
# Ortam degiskenleri:
#   AVTECH_TEENSY_PORT=...   Teensy portunu sabitle (varsayilan: otomatik, PJRC VID)
#   AVTECH_WEB_PORT=5000

import math
import os
import signal
import threading
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(BASE_DIR)

from flask import Flask, send_from_directory, request
from flask_socketio import SocketIO, emit

from teensy_link import TeensyLink

# --- Konfigürasyon ---

WEB_PORT = int(os.environ.get('AVTECH_WEB_PORT', '5000'))
WEB_HOST = '0.0.0.0'

# Hız modları (FOC birimi, -1000..+1000). Teensy config.h sınırları: MOTOR_SPEED_MAX=400, MOTOR_STEER_MAX=250.
# speed = ileri/geri tam gaz, steer = tam direksiyon (yerinde dönüşte de bu kullanılır).
SPEED_MODES = {
    'yavas': {'label': 'YAVAŞ', 'speed': 120, 'steer': 90},
    'orta':  {'label': 'ORTA',  'speed': 220, 'steer': 140},
    'hizli': {'label': 'HIZLI', 'speed': 350, 'steer': 200},
}
MODE_ORDER = ['yavas', 'orta', 'hizli']
DEFAULT_MODE = 'yavas'      # Açılışta her zaman güvenli mod

DRIVE_HZ = 20               # Teensy'ye manual komut hızı (Teensy 500 ms'de komut gelmezse durur)
GUI_DEADMAN_S = 0.6         # GUI sürüş paketi bu kadar gelmezse hedef = 0
ZERO_TAIL_S = 0.5           # Bırakınca bu süre boyunca 0 gönderilmeye devam (kayıp pakete karşı)
AXIS_DEADZONE = 0.05        # Sunucu tarafı ölü bölge (GUI ayrıca uygular)

FORK_JOG_LIMIT = 2000       # config.h FORK_JOG_MAX_STEPS ile aynı
STATUS_HZ = 5

# --- Flask App ---

GUI_DIR = os.path.normpath(os.path.join(BASE_DIR, '..', 'gui'))
app = Flask(__name__, static_folder=None)
app.config['SECRET_KEY'] = 'avtech-agv-2026'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# --- Sürüş durumu ---

drive_lock = threading.RLock()   # Hedef okuma + Teensy'ye gönderim tek parça: bırakmadan sonra bayat komut gitmesin
drive_x = 0.0               # Direksiyon ekseni (-1 sol .. +1 sağ)
drive_y = 0.0               # Gaz ekseni (-1 geri .. +1 ileri)
drive_ts = 0.0              # Son GUI sürüş paketi (monotonic)
drive_sid = None            # Süren GUI istemcisi
speed_mode = DEFAULT_MODE
last_cmd = {'speed': 0, 'steer': 0}
deadman_tripped = False


def ui_log(msg, level='muted'):
    print(msg, flush=True)
    socketio.emit('log', {'msg': msg, 'level': level})


# --- Teensy ---

def on_teensy_json(data):
    if 'state' not in data:
        socketio.emit('teensy_event', data)
        if data.get('resp') == 'rejected':
            ui_log(f"[TEENSY] Komut reddedildi: {data.get('cmd')} ({data.get('reason')})", 'orange')
        elif data.get('resp') == 'ready':
            ui_log("[TEENSY] Teensy yeniden başladı (çatal alta iniyor, IMU kalibre edildi)", 'orange')
        elif data.get('status') == 'link_lost':
            ui_log("[TEENSY] Jetson kalp atışı kesildi — Teensy hareketi durdurdu", 'red')
        elif data.get('status') == 'hold_diverge':
            ui_log(f"[IMU] Yön tutma hatası {data.get('err')}° — hedef sıfırlandı "
                   "(sürekli oluyorsa IMU_YAW_INVERT'i kontrol edin)", 'orange')
        return

    enriched = dict(data)
    enriched['drive'] = dict(last_cmd)
    enriched['speedMode'] = speed_mode
    socketio.emit('telemetry', enriched)


def on_teensy_text(line):
    if line.startswith('[DBG]'):
        return
    print(f"[RX] {line}", flush=True)
    socketio.emit('log', {'msg': line, 'level': 'muted'})


teensy = TeensyLink(on_json=on_teensy_json, on_text=on_teensy_text)


def send(cmd, quiet=False):
    if not teensy.port_open:
        return False
    return teensy.send(cmd, quiet=quiet)


# --- Yardımcılar ---

def _num(v, lo=-1.0, hi=1.0):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(f):
        return 0.0
    return max(lo, min(hi, f))


def _int(v, default=0):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    return int(round(f)) if math.isfinite(f) else default


def _deadzone(v):
    if abs(v) < AXIS_DEADZONE:
        return 0.0
    return math.copysign((abs(v) - AXIS_DEADZONE) / (1.0 - AXIS_DEADZONE), v)


def axes_to_cmd(x, y, mode):
    m = SPEED_MODES[mode]
    return {'speed': int(round(_deadzone(y) * m['speed'])),
            'steer': int(round(_deadzone(x) * m['steer']))}


def clear_drive(send_zero=False):
    global drive_x, drive_y, drive_sid
    with drive_lock:
        drive_x = drive_y = 0.0
        drive_sid = None
        if send_zero:
            send({"cmd": "manual", "speed": 0, "steer": 0}, quiet=True)


# --- Sürüş döngüsü ---

def drive_loop():
    """Sabit 20 Hz: hedef eksenleri Teensy manual komutuna çevirir."""
    global last_cmd, deadman_tripped, drive_x, drive_y
    period = 1.0 / DRIVE_HZ
    last_active = 0.0
    while True:
        t0 = time.monotonic()
        try:
            with drive_lock:
                x, y, ts = drive_x, drive_y, drive_ts
                if (x or y) and t0 - ts > GUI_DEADMAN_S:
                    drive_x = drive_y = 0.0
                    x = y = 0.0
                    deadman_tripped = True

                cmd = axes_to_cmd(x, y, speed_mode)
                last_cmd = cmd
                if cmd['speed'] or cmd['steer']:
                    last_active = t0
                    send({"cmd": "manual", "speed": cmd['speed'], "steer": cmd['steer']}, quiet=True)
                elif t0 - last_active < ZERO_TAIL_S:
                    send({"cmd": "manual", "speed": 0, "steer": 0}, quiet=True)
            if deadman_tripped:
                deadman_tripped = False
                ui_log("[SÜRÜŞ] GUI sinyali kesildi — araç durduruluyor", 'red')
        except Exception as e:   # Döngü asla ölmesin
            print(f"[SÜRÜŞ] Döngü hatası: {e}", flush=True)
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


def system_status():
    return {
        'teensy': teensy.status_dict(),
        'mode': speed_mode,
        'driving': bool(last_cmd['speed'] or last_cmd['steer']),
    }


def status_thread():
    while True:
        try:
            socketio.emit('system_status', system_status())
        except Exception as e:
            print(f"[BRAIN] Durum yayını hatası: {e}", flush=True)
        time.sleep(1.0 / STATUS_HZ)


# --- Web Routes ---

@app.route('/')
def index():
    return send_from_directory(GUI_DIR, 'index.html')


@app.route('/api/status')
def api_status():
    st = system_status()
    st['telemetry'] = teensy.telemetry
    return st


@app.route('/<path:filename>')
def static_files(filename):
    return send_from_directory(GUI_DIR, filename)


# --- WebSocket Events ---

@socketio.on('connect')
def handle_connect(*_):
    print(f"[WS] GUI bağlandı ({request.sid})", flush=True)
    emit('config', {
        'modes': {k: SPEED_MODES[k] for k in MODE_ORDER},
        'order': MODE_ORDER,
        'mode': speed_mode,
        'deadman_ms': int(GUI_DEADMAN_S * 1000),
    })
    emit('system_status', system_status())


@socketio.on('disconnect')
def handle_disconnect(*_):
    print(f"[WS] GUI ayrıldı ({request.sid})", flush=True)
    if request.sid == drive_sid:
        clear_drive(send_zero=True)
        print("[WS] Süren GUI ayrıldı — motorlar 0", flush=True)


@socketio.on('drive')
def handle_drive(data):
    global drive_x, drive_y, drive_ts, drive_sid
    data = data if isinstance(data, dict) else {}
    x, y = _num(data.get('x')), _num(data.get('y'))
    with drive_lock:
        drive_x, drive_y = x, y
        drive_ts = time.monotonic()
        drive_sid = request.sid if (x or y) else drive_sid


@socketio.on('drive_release')
def handle_drive_release(*_):
    clear_drive(send_zero=True)


@socketio.on('set_mode')
def handle_set_mode(data):
    global speed_mode
    mode = (data or {}).get('mode') if isinstance(data, dict) else None
    if mode not in SPEED_MODES:
        return
    if mode != speed_mode:
        speed_mode = mode
        ui_log(f"[SÜRÜŞ] Hız modu: {SPEED_MODES[mode]['label']}", 'cyan')
    socketio.emit('mode', {'mode': speed_mode})


@socketio.on('stop')
def handle_stop(*_):
    with drive_lock:
        clear_drive(send_zero=True)
        ok = send({"cmd": "stop"})
    ui_log("[STOP] Yazılımsal STOP — sürüş ve çatal durduruldu" if ok else
           "[STOP] Teensy bağlı değil!", 'red')


@socketio.on('reset')
def handle_reset(*_):
    clear_drive()
    if send({"cmd": "reset"}):
        ui_log("[TEENSY] Reset gönderildi", 'cyan')


@socketio.on('fork')
def handle_fork(data):
    data = data if isinstance(data, dict) else {}
    action = data.get('action')
    if action in ('up', 'down', 'stop', 'zero'):
        cmd = {"cmd": "forklift", "action": action}
    elif action == 'hold':
        d = _int(data.get('dir'))
        cmd = {"cmd": "forklift", "action": "hold", "dir": 1 if d > 0 else (-1 if d < 0 else 0)}
    elif action == 'jog':
        steps = max(-FORK_JOG_LIMIT, min(FORK_JOG_LIMIT, _int(data.get('steps'))))
        if steps == 0:
            return
        cmd = {"cmd": "forklift", "action": "jog", "steps": steps}
        if data.get('nolimit'):
            cmd['nolimit'] = 1
    else:
        return
    quiet = action == 'hold'
    if not send(cmd, quiet=quiet) and not quiet:
        emit('log', {'msg': '[ÇATAL] Teensy bağlı değil', 'level': 'red'})


@socketio.on('hold_enable')
def handle_hold_enable(data):
    on = bool((data or {}).get('on')) if isinstance(data, dict) else True
    if send({"cmd": "hold", "on": 1 if on else 0}):
        ui_log(f"[IMU] Düz gidişte yön tutma {'AÇIK' if on else 'KAPALI'}", 'cyan')


@socketio.on('sync_yaw')
def handle_sync_yaw(data):
    yaw = _int((data or {}).get('yaw') if isinstance(data, dict) else 0) % 360
    if send({"cmd": "sync_yaw", "yaw": yaw}):
        ui_log(f"[IMU] Yaw = {yaw}° olarak ayarlandı", 'cyan')


@socketio.on('ping_rt')
def handle_ping_rt(*_):
    return {'t': time.time()}


# --- Ana Program ---

def main():
    print("=" * 60)
    print("  AVTECH AGV — Manuel Kontrol Sunucusu v5")
    print("  GUI (joystick + WASD) → Teensy 4.1 (Hoverboard | ICM-20948 | DM556)")
    print("=" * 60)

    teensy.start()
    threading.Thread(target=drive_loop, daemon=True, name='drive').start()
    threading.Thread(target=status_thread, daemon=True, name='status').start()

    def _sigterm(*_):
        raise KeyboardInterrupt   # systemd stop → finally bloğu motorları durdursun
    signal.signal(signal.SIGTERM, _sigterm)

    print(f"[WEB] GUI: http://<jetson-ip>:{WEB_PORT}")
    try:
        socketio.run(app, host=WEB_HOST, port=WEB_PORT, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        print("[BRAIN] Kapanıyor — motorlar durduruluyor")
        clear_drive()
        for _ in range(3):
            teensy.send({"cmd": "manual", "speed": 0, "steer": 0}, quiet=True)
            teensy.send({"cmd": "stop"}, quiet=True)
            time.sleep(0.02)


if __name__ == '__main__':
    main()
