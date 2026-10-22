#!/usr/bin/env python3
# AVTECH AGV - Teensy 4.1 USB Seri Baglantisi (Jetson tarafi)
# avtech_brain.py ve rc_drive.py ortak kullanir.
#
# - Teensy portunu PJRC VID (0x16C0) ile otomatik bulur, kopunca yeniden baglanir
#   (Teensy yeniden takilinca ttyACM0 → ttyACM1 olsa bile).
# - Portu Linux'ta ozel (exclusive) acar: ikinci bir program ayni portu acamaz,
#   boylece iki surec ayni anda motor komutu gonderemez.
# - Komutlari kompakt JSON satiri olarak gonderir: {"cmd":"manual","speed":0,"steer":0}
# - 5 Hz kalp atisi ({"cmd":"hb"}) gonderir; Teensy bu kesilirse hareketi durdurur.
#
# Tek basina test:  python3 teensy_link.py   (telemetriyi yazdirir, ping atar)

import json
import os
import threading
import time

import serial
import serial.tools.list_ports

PJRC_VID = 0x16C0
DEFAULT_BAUD = 115200
RX_TIMEOUT_S = 1.5          # Bu kadar satir gelmezse baglanti "kopuk" sayilir
HEARTBEAT_PERIOD_S = 0.2    # 5 Hz (Teensy JETSON_LINK_TIMEOUT_MS = 1000)
WRITE_TIMEOUT_S = 0.3


def _compact(cmd):
    return json.dumps(cmd, separators=(',', ':'))


def find_teensy_port(exclude=()):
    """Teensy'yi bul. Once PJRC VID/aciklama, yoksa tek bir ttyACM varsa onu dondur."""
    exclude = set(p for p in exclude if p)
    ports = [p for p in serial.tools.list_ports.comports() if p.device not in exclude]
    for p in ports:
        desc = (p.description or '').lower()
        mfg = (p.manufacturer or '').lower()
        if p.vid == PJRC_VID or 'teensy' in desc or 'teensy' in mfg or 'pjrc' in mfg:
            return p.device
    acm = [p.device for p in ports if 'ttyACM' in p.device]
    if len(acm) == 1:
        return acm[0]
    return None


class TeensyLink:
    """Teensy USB seri baglantisi.

    on_json(dict) : her JSON satiri icin (telemetri, resp, status) — okuyucu thread'inde cagrilir
    on_text(str)  : JSON olmayan satirlar (debug loglari)
    exclude_ports : callable → set; otomatik tarama sirasinda atlanacak portlar (kumanda alicisi gibi)
    """

    def __init__(self, port=None, baud=DEFAULT_BAUD, on_json=None, on_text=None,
                 exclude_ports=None, heartbeat=True, verbose=True):
        self.fixed_port = port or os.environ.get('AVTECH_TEENSY_PORT') or None
        self.baud = baud
        self.on_json = on_json
        self.on_text = on_text
        self.exclude_ports = exclude_ports or (lambda: set())
        self.heartbeat = heartbeat
        self.verbose = verbose

        self.port = None
        self._ser = None
        self._tx_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._running = False

        self.telemetry = {}
        self.last_rx = 0.0
        self.last_telemetry_ts = 0.0
        self.tx_errors = 0
        self.reconnects = 0

    # ── Durum ──

    @property
    def port_open(self):
        return self._ser is not None

    @property
    def connected(self):
        """Port acik VE yakin zamanda veri geldi."""
        return self._ser is not None and (time.monotonic() - self.last_rx) < RX_TIMEOUT_S

    def status_dict(self):
        return {
            'connected': self.connected,
            'port_open': self.port_open,
            'port': self.port,
            'rx_age_ms': None if not self.last_rx else int((time.monotonic() - self.last_rx) * 1000),
            'tx_errors': self.tx_errors,
            'reconnects': self.reconnects,
        }

    # ── Yasam dongusu ──

    def start(self):
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._reader, daemon=True, name='teensy_rx').start()
        if self.heartbeat:
            threading.Thread(target=self._heartbeat, daemon=True, name='teensy_hb').start()

    def close(self):
        self._running = False
        self._drop('kapatildi')

    # ── Gonderim ──

    def send(self, cmd, quiet=False):
        """Komutu kompakt JSON satiri olarak gonder. Basariliysa True."""
        ser = self._ser
        if ser is None:
            return False
        line = _compact(cmd)
        try:
            with self._tx_lock:
                ser.write((line + '\n').encode('utf-8'))
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as e:
            self.tx_errors += 1
            self._log(f"Gonderim hatasi: {e}")
            self._drop(str(e))
            return False
        if not quiet:
            self._log(f"TX {line}")
        return True

    # ── Dahili ──

    def _log(self, msg):
        if self.verbose:
            print(f"[TEENSY] {msg}", flush=True)

    def _open(self):
        port = self.fixed_port or find_teensy_port(self.exclude_ports())
        if not port:
            return False
        kwargs = dict(baudrate=self.baud, timeout=0.05, write_timeout=WRITE_TIMEOUT_S)
        if os.name == 'posix':
            kwargs['exclusive'] = True
        try:
            ser = serial.Serial(port, **kwargs)
            ser.reset_input_buffer()
        except (serial.SerialException, OSError, ValueError) as e:
            self._log(f"Port acilamadi ({port}): {e}")
            return False
        with self._state_lock:
            self._ser = ser
            self.port = port
            self.last_rx = time.monotonic()   # Acilis toleransi
        self.reconnects += 1
        self._log(f"Baglandi: {port} @ {self.baud}")
        return True

    def _drop(self, why=''):
        with self._state_lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
            if why:
                self._log(f"Baglanti kapandi: {why}")

    def _heartbeat(self):
        while self._running:
            if self._ser is not None:
                self.send({"cmd": "hb"}, quiet=True)
            time.sleep(HEARTBEAT_PERIOD_S)

    def _reader(self):
        buf = bytearray()
        while self._running:
            ser = self._ser
            if ser is None:
                buf.clear()
                if not self._open():
                    time.sleep(1.0)
                continue
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError) as e:
                self._drop(f"okuma hatasi: {e}")
                time.sleep(1.0)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            if len(buf) > 8192 and b'\n' not in buf:
                buf.clear()   # Cop veri: satir sonu yok
                continue
            while True:
                nl = buf.find(b'\n')
                if nl < 0:
                    break
                raw = bytes(buf[:nl])
                del buf[:nl + 1]
                line = raw.decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                self.last_rx = time.monotonic()
                self._dispatch(line)

    def _dispatch(self, line):
        if line.startswith('{'):
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                data = None
            if isinstance(data, dict):
                if 'state' in data:
                    self.telemetry = data
                    self.last_telemetry_ts = time.monotonic()
                if self.on_json:
                    try:
                        self.on_json(data)
                    except Exception as e:   # Geri cagri hatasi okuyucuyu oldurmesin
                        print(f"[TEENSY] on_json hatasi: {e}", flush=True)
                return
        if self.on_text:
            try:
                self.on_text(line)
            except Exception as e:
                print(f"[TEENSY] on_text hatasi: {e}", flush=True)


if __name__ == '__main__':
    def _j(d):
        if 'state' in d:
            print(f"\r[TLM] st={d.get('state')} yaw={d.get('yaw')} fork={d.get('fork')}/{d.get('forkSt')} "
                  f"bat={d.get('bat')}V mot={d.get('motL')}/{d.get('motR')} estop={d.get('estop')}   ",
                  end='', flush=True)
        else:
            print(f"\n[EVT] {d}")

    link = TeensyLink(on_json=_j, on_text=lambda s: print(f"\n[LOG] {s}"))
    link.start()
    try:
        while True:
            time.sleep(2.0)
            link.send({"cmd": "ping"}, quiet=True)
    except KeyboardInterrupt:
        link.send({"cmd": "stop"})
        link.close()
        print()
