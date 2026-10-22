#!/usr/bin/env python3
# AVTECH AGV - RC Kumanda Alicisi
# Kumanda alicisindan (FlySky iBUS veya Flipsky/VESC UART) gelen kanallari okur,
# RC araba tarzi diferansiyel surus miksini yapar ve sinyal kaybinda failsafe'e gecer.
#
# Protokol secimi (ortam degiskeni AVTECH_RC_PROTOCOL):
#   ibus      → FlySky FS-i6 / FS-i6X + FS-iA6B/iA10B alici, iBUS pini → USB-TTL RX (varsayilan)
#   vesc_uart → Flipsky VX1/VX2/VX3 alicisi UART modunda (VESC nunchuk paketi)
#               Not: Flipsky kumandalarin cogunda tek eksen (gaz) vardir; direksiyon ekseni
#               yoksa arac yalnizca ileri/geri gider. C butonu = forklift yukari, Z = asagi.
#
# Bench testi (Jetson uzerinde, avtech_brain calismiyorken):
#   python3 rc_receiver.py            -> canli kanal / miks degerlerini yazdirir
#   python3 rc_receiver.py --selftest -> donanimsiz parser + mikser testleri

import os
import sys
import time
import struct
import threading
from dataclasses import dataclass

import serial

# --- Konfigürasyon ---
# Ortam degiskenleriyle de ezilebilir (AVTECH_RC_PROTOCOL, AVTECH_RC_PORT, AVTECH_RC_BAUD)

RC_ENABLED  = os.environ.get('AVTECH_RC_ENABLED', '1') == '1'
RC_PROTOCOL = os.environ.get('AVTECH_RC_PROTOCOL', 'ibus')       # 'ibus' | 'vesc_uart'
RC_PORT     = os.environ.get('AVTECH_RC_PORT', 'auto')   # 'auto' = iBUS akan portu tara
RC_BAUD     = int(os.environ.get('AVTECH_RC_BAUD', '115200'))

# iBUS kanal eslesmesi (0-tabanli indeks). FS-i6 Mode 2 varsayimi:
#   CH1 = sag cubuk yatay, CH2 = sag cubuk dikey (yaylı, ortaya doner), CH5 = SwA
RC_CH_THROTTLE = 1      # CH2 → ileri/geri
RC_CH_STEERING = 0      # CH1 → sag/sol
RC_CH_ARM      = 4      # CH5 → arm switch (None = arm switch yok, her zaman armed)
RC_ARM_THRESHOLD_US = 1700   # Arm kanali bu degerin ustundeyse ARMED

RC_INVERT_THROTTLE = os.environ.get('AVTECH_RC_INVERT_THROTTLE', '0') == '1'
RC_INVERT_STEERING = os.environ.get('AVTECH_RC_INVERT_STEERING', '0') == '1'

# Ek kanallar (iBUS, 0-tabanli). None = kullanilmaz.
RC_CH_FORKLIFT   = 5            # CH6 SwB (2 konum): yukari ↔ asagi gecisi forklift'i bir kez tetikler
RC_CH_SPEED_MODE = 6            # CH7 SwC (3 konum): yavas / orta / hizli
SPEED_MODE_SCALE = (0.4, 0.7, 1.0)

# Failsafe / filtre
RC_TIMEOUT_S        = 0.25   # Bu sureden eski frame = sinyal yok → motor 0
RC_DEADBAND         = 0.05   # Cubuk ortasindaki olu bolge (normalize)
RC_EXPO             = 0.30   # 0 = dogrusal, 1 = tam kubik (merkezde hassas kontrol)
RC_ARM_NEUTRAL_MAX  = 0.10   # Arm anında gaz bu degerin altinda olmali (kacak arac engeli)

# Teensy birimlerinde manuel limitler (config.h: MOTOR_SPEED_MAX=400, MOTOR_STEER_MAX=250)
RC_SPEED_LIMIT = 300
RC_STEER_LIMIT = 200


# --- Miks Matematigi ---

def apply_deadband_expo(x, deadband=RC_DEADBAND, expo=RC_EXPO):
    """[-1, 1] eksen degerine olu bolge + expo egrisi uygula."""
    if abs(x) <= deadband:
        return 0.0
    sign = 1.0 if x > 0 else -1.0
    x = min((abs(x) - deadband) / (1.0 - deadband), 1.0)
    x = (1.0 - expo) * x + expo * x ** 3
    return sign * x


def mix_differential(throttle, steering):
    """RC araba / tank miksi: gaz + direksiyon → sol/sag tekerlek ([-1, 1]).

    Doyumda iki tekerlegi ayni oranla olcekler; boylece tam gaz + tam direksiyonda
    donus egriligi korunur (tek tekerlegi kirpmak yerine).
    """
    left = throttle + steering
    right = throttle - steering
    m = max(1.0, abs(left), abs(right))
    return left / m, right / m


def to_teensy_manual(left, right, speed_limit=RC_SPEED_LIMIT, steer_limit=RC_STEER_LIMIT):
    """Sol/sag tekerlek → Teensy'nin bekledigi {speed, steer} ciftine donustur.

    Hoverboard FOC firmware'i tekrar L = speed + steer, R = speed - steer miksini yapar.
    mix_differential |speed|+|steer| <= 1 garantiledigi icin FOC tarafinda doyum olmaz.
    """
    speed = (left + right) / 2.0 * speed_limit
    steer = (left - right) / 2.0 * steer_limit
    return int(round(speed)), int(round(steer))


# --- Protokol Parser'lari ---

class IBusParser:
    """FlySky iBUS: 115200 8N1, 32 bayt frame.
    [0x20][0x40][14 x uint16 LE kanal][uint16 LE checksum = 0xFFFF - sum(bayt 0..29)]
    """
    FRAME_LEN = 32
    NUM_CHANNELS = 14

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self._buf.extend(data)
        frames = []
        while len(self._buf) >= self.FRAME_LEN:
            if self._buf[0] != 0x20 or self._buf[1] != 0x40:
                del self._buf[0]
                continue
            frame = bytes(self._buf[:self.FRAME_LEN])
            chk = struct.unpack_from('<H', frame, 30)[0]
            if (0xFFFF - sum(frame[:30])) & 0xFFFF != chk:
                self.crc_errors += 1
                del self._buf[0]
                continue
            del self._buf[:self.FRAME_LEN]
            frames.append(list(struct.unpack_from('<14H', frame, 2)))
        if len(self._buf) > 256:
            del self._buf[:-self.FRAME_LEN]
        return frames


def crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


class VescChuckParser:
    """Flipsky (VX1/VX2...) alicisinin UART modu: VESC paket protokolu.
    [0x02][len][payload][crc16 BE][0x03]  (len > 255 ise [0x03][len16]...)
    payload[0] = COMM_SET_CHUCK_DATA (35): js_x, js_y, bt_c, bt_z, acc_x/y/z (int16)
    Diger paketler (ornek: COMM_GET_VALUES istegi) yok sayilir.
    """
    COMM_SET_CHUCK_DATA = 35

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self._buf.extend(data)
        frames = []
        while True:
            while self._buf and self._buf[0] not in (0x02, 0x03):
                del self._buf[0]
            if len(self._buf) < 2:
                break
            if self._buf[0] == 0x02:
                hdr, plen = 2, self._buf[1]
            else:
                if len(self._buf) < 3:
                    break
                hdr, plen = 3, (self._buf[1] << 8) | self._buf[2]
            total = hdr + plen + 3
            if plen == 0 or plen > 512:
                del self._buf[0]
                continue
            if len(self._buf) < total:
                break
            payload = bytes(self._buf[hdr:hdr + plen])
            crc = (self._buf[hdr + plen] << 8) | self._buf[hdr + plen + 1]
            if self._buf[total - 1] != 0x03 or crc16_xmodem(payload) != crc:
                self.crc_errors += 1
                del self._buf[0]
                continue
            del self._buf[:total]
            if payload[0] == self.COMM_SET_CHUCK_DATA and len(payload) >= 5:
                js_x, js_y, bt_c, bt_z = payload[1], payload[2], payload[3], payload[4]
                frames.append((js_x, js_y, bt_c, bt_z))
        if len(self._buf) > 1024:
            self._buf.clear()
        return frames


# --- Alici Thread'i ---

@dataclass
class RCCommand:
    valid: bool          # Taze ve gecerli frame var mi
    armed: bool          # Arm switch acik + notr-arm sarti saglandi
    throttle: float      # [-1, 1] (deadband/expo uygulanmis)
    steering: float      # [-1, 1]
    reason: str          # Gecersiz/disarmed ise nedeni
    age_s: float         # Son frame'in yasi


class RCReceiver(threading.Thread):
    """Arka planda seri porttan kumanda frame'lerini okur.

    Failsafe katmanlari:
      1) Frame RC_TIMEOUT_S'den eskiyse → valid=False
      2) Seri port hatasi/kopma → valid=False, port kapatilip yeniden acilir
      3) iBUS kanal degeri 900-2100us disinda → frame reddedilir
      4) Arm switch kapali → armed=False
      5) Arm anında gaz notr degilse arm edilmez (kacak arac engeli)
    """

    def __init__(self, port=RC_PORT, baud=RC_BAUD, protocol=RC_PROTOCOL, exclude_ports=None):
        super().__init__(daemon=True, name='rc_receiver')
        if protocol not in ('ibus', 'vesc_uart'):
            raise ValueError(f"Bilinmeyen RC protokolu: {protocol}")
        self.auto_port = (port == 'auto')
        self.port = port
        self.exclude_ports = exclude_ports or (lambda: set())   # Teensy portu taranmasin
        self.baud = baud
        self.protocol = protocol
        self._parser = IBusParser() if protocol == 'ibus' else VescChuckParser()
        self._ser = None
        self._lock = threading.Lock()
        self._last_ts = 0.0
        self._throttle_raw = 0.0
        self._steering_raw = 0.0
        self._arm_switch = False
        self._arm_latched = False
        self._port_ok = False
        self._last_error = ''
        self.raw_channels = []
        self.frame_count = 0
        self._buttons = (False, False)

    # ── Public ──

    def get_command(self):
        """Mikse hazir, failsafe uygulanmis anlik komut."""
        now = time.monotonic()
        with self._lock:
            age = now - self._last_ts if self._last_ts else float('inf')

            reason = ''
            if not self._port_ok:
                reason = f'port: {self._last_error or "kapali"}'
            elif age > RC_TIMEOUT_S:
                reason = 'sinyal yok (timeout)'

            if reason:
                self._arm_latched = False
                return RCCommand(False, False, 0.0, 0.0, reason, age)

            throttle = apply_deadband_expo(self._throttle_raw)
            steering = apply_deadband_expo(self._steering_raw)

            if not self._arm_switch:
                self._arm_latched = False
                return RCCommand(True, False, 0.0, 0.0, 'arm switch kapali', age)

            if not self._arm_latched:
                if abs(self._throttle_raw) > RC_ARM_NEUTRAL_MAX:
                    return RCCommand(True, False, 0.0, 0.0, 'arm icin gazi ortala', age)
                self._arm_latched = True

            return RCCommand(True, True, throttle, steering, '', age)

    def get_channel_us(self, ch):
        """Ham kanal degeri (us). Taze ve gecerli frame yoksa None (switch'ler icin)."""
        with self._lock:
            if not self._last_ts or time.monotonic() - self._last_ts > RC_TIMEOUT_S:
                return None
            if ch is None or ch >= len(self.raw_channels):
                return None
            v = self.raw_channels[ch]
            return v if 900 <= v <= 2100 else None

    def get_buttons(self):
        """VESC/Flipsky (C, Z) butonlari. Taze frame yoksa None."""
        with self._lock:
            if not self._last_ts or time.monotonic() - self._last_ts > RC_TIMEOUT_S:
                return None
            return self._buttons

    def status_dict(self):
        cmd = self.get_command()
        return {
            'protocol': self.protocol,
            'port': self.port,
            'port_ok': self._port_ok,
            'valid': cmd.valid,
            'armed': cmd.armed,
            'reason': cmd.reason,
            'throttle': round(cmd.throttle, 3),
            'steering': round(cmd.steering, 3),
            'age_ms': None if cmd.age_s == float('inf') else int(cmd.age_s * 1000),
            'frames': self.frame_count,
            'crc_errors': self._parser.crc_errors,
        }

    # ── Thread ──

    def run(self):
        while True:
            if self._ser is None:
                if not self._open():
                    time.sleep(1.0)
                    continue
            try:
                data = self._ser.read(self._ser.in_waiting or 1)   # timeout=0.05
                if data:
                    for frame in self._parser.feed(data):
                        self._handle_frame(frame)
            except (serial.SerialException, OSError) as e:
                print(f"[RC] Seri port hatasi: {e} — yeniden baglaniliyor")
                self._close(str(e))
                time.sleep(0.5)

    def _discover_port(self):
        """Aday seri portlari dinle, gecerli frame gelen ilk portu dondur."""
        import glob
        excluded = self.exclude_ports()
        candidates = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyTHS*') +
                            glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyAMA*'))
        for dev in candidates:
            if dev in excluded:
                continue
            parser = IBusParser() if self.protocol == 'ibus' else VescChuckParser()
            try:
                with serial.Serial(dev, self.baud, timeout=0.1) as s:
                    s.reset_input_buffer()
                    t_end = time.monotonic() + 0.4
                    while time.monotonic() < t_end:
                        if parser.feed(s.read(64)):
                            print(f"[RC] Alici bulundu: {dev}")
                            return dev
            except (serial.SerialException, OSError):
                continue
        return None

    def _open(self):
        if self.auto_port:
            dev = self._discover_port()
            if dev is None:
                self._close('alici bulunamadi (port taraniyor)')
                return False
            self.port = dev
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self._ser.reset_input_buffer()
            with self._lock:
                self._port_ok = True
                self._last_error = ''
            print(f"[RC] Alici baglandi: {self.port} @ {self.baud} ({self.protocol})")
            return True
        except (serial.SerialException, OSError) as e:
            self._close(str(e))
            return False

    def _close(self, err):
        with self._lock:
            self._port_ok = False
            self._last_error = err
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    def _handle_frame(self, frame):
        if self.protocol == 'ibus':
            used = [RC_CH_THROTTLE, RC_CH_STEERING] + ([RC_CH_ARM] if RC_CH_ARM is not None else [])
            if any(not (900 <= frame[ch] <= 2100) for ch in used):
                return   # Bozuk/failsafe disi deger → frame'i kullanma (timeout devreye girer)
            thr = (frame[RC_CH_THROTTLE] - 1500) / 500.0
            ste = (frame[RC_CH_STEERING] - 1500) / 500.0
            arm = True if RC_CH_ARM is None else frame[RC_CH_ARM] >= RC_ARM_THRESHOLD_US
        else:
            js_x, js_y, bt_c, bt_z = frame
            thr = (js_y - 128) / 127.0
            ste = (js_x - 128) / 127.0
            arm = True   # VESC chuck verisinde arm kanali yok; failsafe = timeout + notr-arm
            with self._lock:
                self._buttons = (bool(bt_c), bool(bt_z))

        thr = max(-1.0, min(1.0, -thr if RC_INVERT_THROTTLE else thr))
        ste = max(-1.0, min(1.0, -ste if RC_INVERT_STEERING else ste))

        with self._lock:
            self._throttle_raw = thr
            self._steering_raw = ste
            self._arm_switch = arm
            self._last_ts = time.monotonic()
            self.raw_channels = list(frame)
            self.frame_count += 1


# --- Surus + Forklift Mantigi ---

class RCDriveLogic:
    """Kumanda durumunu Teensy komutlarina cevirir (avtech_brain ve rc_drive ortak).

    - drive_command(cmd): gaz/direksiyon → {"cmd":"manual",...} (hiz modu olcekli)
    - forklift_event():   'up' / 'down' / None — switch GECISI veya buton BASISI bir kez tetikler.
      Kontrol devralindiginda reset_edges() cagrilir: o anki switch konumu tetiklemez.
    """

    def __init__(self, rc, speed_limit=None, steer_limit=None):
        self.rc = rc
        self.speed_limit = RC_SPEED_LIMIT if speed_limit is None else speed_limit
        self.steer_limit = RC_STEER_LIMIT if steer_limit is None else steer_limit
        self._fork_prev = None
        self._btn_prev = None

    def _fork_switch(self):
        if RC_CH_FORKLIFT is None:
            return None
        v = self.rc.get_channel_us(RC_CH_FORKLIFT)
        return None if v is None else v >= 1500

    def speed_scale(self):
        if self.rc.protocol != 'ibus' or RC_CH_SPEED_MODE is None:
            return SPEED_MODE_SCALE[1]
        v = self.rc.get_channel_us(RC_CH_SPEED_MODE)
        if v is None:
            return SPEED_MODE_SCALE[1]
        if v < 1300:
            return SPEED_MODE_SCALE[0]
        if v > 1700:
            return SPEED_MODE_SCALE[2]
        return SPEED_MODE_SCALE[1]

    def reset_edges(self):
        self._fork_prev = self._fork_switch() if self.rc.protocol == 'ibus' else None
        self._btn_prev = self.rc.get_buttons() if self.rc.protocol == 'vesc_uart' else None

    def forklift_event(self):
        if self.rc.protocol == 'ibus':
            now = self._fork_switch()
            ev = None
            if now is not None and self._fork_prev is not None and now != self._fork_prev:
                ev = 'up' if now else 'down'
            if now is not None:
                self._fork_prev = now
            return ev
        # vesc_uart: butonun basilma ani (yukselen kenar)
        now = self.rc.get_buttons()
        ev = None
        if now is not None:
            prev = self._btn_prev or (False, False)
            if now[0] and not prev[0]:
                ev = 'up'
            elif now[1] and not prev[1]:
                ev = 'down'
            self._btn_prev = now
        return ev

    def drive_command(self, cmd):
        if not cmd.armed:
            return {"cmd": "manual", "speed": 0, "steer": 0}
        left, right = mix_differential(cmd.throttle, cmd.steering)
        k = self.speed_scale()
        speed, steer = to_teensy_manual(left, right, self.speed_limit * k, self.steer_limit * k)
        return {"cmd": "manual", "speed": speed, "steer": steer}


# --- Testler ---

def _selftest():
    # iBUS
    ch = [1500] * 14
    ch[RC_CH_THROTTLE] = 2000
    ch[RC_CH_STEERING] = 1500
    body = bytes([0x20, 0x40]) + struct.pack('<14H', *ch)
    frame = body + struct.pack('<H', (0xFFFF - sum(body)) & 0xFFFF)
    p = IBusParser()
    assert p.feed(b'\x00\x13' + frame[:10]) == []
    out = p.feed(frame[10:] + frame)
    assert len(out) == 2 and out[0][RC_CH_THROTTLE] == 2000, out
    bad = bytearray(frame); bad[5] ^= 0xFF
    assert p.feed(bytes(bad)) == [] and p.crc_errors >= 1

    # VESC chuck
    payload = bytes([35, 128, 255, 0, 0]) + struct.pack('>hhh', 0, 0, 0)
    crc = crc16_xmodem(payload)
    pkt = bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])
    vp = VescChuckParser()
    assert vp.feed(b'\xAA' + pkt[:4]) == []
    assert vp.feed(pkt[4:]) == [(128, 255, 0, 0)]

    # Mikser
    assert mix_differential(1, 0) == (1, 1)
    assert mix_differential(0, 1) == (1, -1)                 # Yerinde saga donus
    l, r = mix_differential(1, 1)
    assert (l, r) == (1.0, 0.0)                              # Doyumda oran korunur
    assert to_teensy_manual(1, 1) == (RC_SPEED_LIMIT, 0)
    assert to_teensy_manual(1, -1) == (0, RC_STEER_LIMIT)
    assert apply_deadband_expo(0.03) == 0.0
    assert abs(apply_deadband_expo(1.0) - 1.0) < 1e-9

    # Failsafe: frame yoksa gecersiz
    rx = RCReceiver(port='NONE')
    assert not rx.get_command().valid
    assert rx.get_buttons() is None

    # Forklift switch gecisi: ilk deger tetiklemez, gecis bir kez tetikler
    rx._port_ok = True
    ch = [1500] * 14
    ch[RC_CH_ARM] = 2000
    ch[RC_CH_FORKLIFT] = 1000
    rx._handle_frame(ch)
    logic = RCDriveLogic(rx)
    logic.reset_edges()
    assert logic.forklift_event() is None
    ch[RC_CH_FORKLIFT] = 2000
    rx._handle_frame(ch)
    assert logic.forklift_event() == 'up'
    assert logic.forklift_event() is None
    ch[RC_CH_FORKLIFT] = 1000
    rx._handle_frame(ch)
    assert logic.forklift_event() == 'down'
    c = rx.get_command()
    assert c.valid and c.armed
    assert logic.drive_command(c) == {"cmd": "manual", "speed": 0, "steer": 0}
    print("[RC] selftest OK")


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        _selftest()
        sys.exit(0)

    rx = RCReceiver()
    rx.start()
    print("[RC] Canli izleme — Ctrl+C ile cik")
    try:
        while True:
            cmd = rx.get_command()
            l, r = mix_differential(cmd.throttle, cmd.steering) if cmd.armed else (0.0, 0.0)
            spd, ste = to_teensy_manual(l, r)
            print(f"\rraw={rx.raw_channels[:6]} valid={cmd.valid} armed={cmd.armed} "
                  f"thr={cmd.throttle:+.2f} ste={cmd.steering:+.2f} L={l:+.2f} R={r:+.2f} "
                  f"-> speed={spd:+4d} steer={ste:+4d} {cmd.reason:<24}", end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
