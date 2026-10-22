#!/usr/bin/env python3
# AVTECH AGV - Kumanda ile Manuel Surus (Jetson, GUI'siz yedek mod)
# Kumanda (FlySky iBUS veya Flipsky VESC UART) → RC diferansiyel miks → Teensy JSON komutlari.
# Teensy: hoverboard FOC (Serial1), ICM-20948 IMU (SPI), DM556 step (forklift).
#
# Normal kullanimda avtech_brain.py (GUI + kumanda birlikte) calistirilir. Bu betik,
# GUI/kamera olmadan sadece kumandayla surmek icin yedek moddur. Teensy portu ozel
# (exclusive) acildigi icin ikisi ayni anda calisamaz; ikincisi port bekler.
#
# Kanal eslesmesi (FS-i6X Mode 2, 0-tabanli indeks rc_receiver.py'de):
#   CH1  sag cubuk yatay  → direksiyon
#   CH2  sag cubuk dikey  → gaz (ileri/geri)
#   CH5  SwA (2 konum)    → ARM  (asagi = motorlar kilitli)
#   CH6  SwB (2 konum)    → forklift  (yukari ↔ asagi gecisi bir kez tetikler)
#   CH7  SwC (3 konum)    → hiz modu  (yavas / orta / hizli)
# Flipsky (vesc_uart): gaz tekerlegi → ileri/geri, C butonu → forklift yukari, Z → asagi
#
# Calistirma:  python3 rc_drive.py            (Ctrl+C ile guvenli cikis)

import signal
import sys
import time

import rc_receiver
from rc_receiver import RCReceiver, RCDriveLogic
from teensy_link import TeensyLink

LOOP_HZ = 50
STATUS_PRINT_S = 0.5


def main():
    def on_json(d):
        if d.get('status') == 'forklift_done':
            print(f"[TEENSY] Forklift islemi tamamlandi (konum {d.get('pos')})")
        elif d.get('resp') == 'rejected':
            print(f"[TEENSY] Reddedildi: {d.get('cmd')} ({d.get('reason')})")
        elif d.get('status') == 'link_lost':
            print("[TEENSY] Kalp atisi kesildi — Teensy hareketi durdurdu")

    def on_text(line):
        if not line.startswith('[DBG]'):
            print(f"[TEENSY] {line}")

    rc = RCReceiver()
    teensy = TeensyLink(on_json=on_json, on_text=on_text,
                        exclude_ports=lambda: {rc.port} if rc._port_ok else set())
    rc.exclude_ports = lambda: {teensy.port} if teensy.port else set()
    logic = RCDriveLogic(rc)
    teensy.start()
    rc.start()

    in_control = False
    last_print = 0.0
    last_reason = None
    period = 1.0 / LOOP_HZ

    def shutdown(*_):
        print("\n[DRIVE] Cikis — motorlar durduruluyor")
        for _ in range(3):
            teensy.send({"cmd": "manual", "speed": 0, "steer": 0}, quiet=True)
            teensy.send({"cmd": "stop"}, quiet=True)
            time.sleep(0.02)
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("=" * 60)
    print("  AVTECH AGV — Kumanda ile Manuel Surus (GUI'siz)")
    print(f"  RC: {rc_receiver.RC_PROTOCOL} @ {rc.port}   Teensy: otomatik")
    if rc.protocol == 'ibus':
        print("  ARM: SwA (CH5)  Forklift: SwB (CH6)  Hiz modu: SwC (CH7)")
    else:
        print("  Flipsky: C butonu = forklift yukari, Z butonu = asagi")
    print("=" * 60)

    while True:
        t0 = time.monotonic()
        cmd = rc.get_command()
        link_ok = teensy.connected
        want = cmd.armed and link_ok

        if cmd.reason != last_reason:
            print(f"[RC] {cmd.reason or 'ARMED — surus aktif'}")
            last_reason = cmd.reason

        if want and not in_control:
            in_control = True
            logic.reset_edges()                       # Arm anindaki switch konumu tetiklemez
            teensy.send({"cmd": "stop"})              # FAULT/DONE'dan IDLE'a cek, MANUAL kabul etsin

        if in_control:
            if want:
                ev = logic.forklift_event()
                if ev:
                    teensy.send({"cmd": "forklift", "action": ev})
                teensy.send(logic.drive_command(cmd), quiet=True)
            else:
                teensy.send({"cmd": "manual", "speed": 0, "steer": 0}, quiet=True)
                teensy.send({"cmd": "stop"})
                in_control = False
                why = cmd.reason if not cmd.armed else 'Teensy baglantisi yok'
                print(f"[DRIVE] FAILSAFE — motorlar durduruldu ({why})")

        if t0 - last_print > STATUS_PRINT_S:
            tm = teensy.telemetry
            print(f"[DURUM] RC:{'OK' if cmd.valid else 'YOK'}/{'ARM' if cmd.armed else 'dis'} "
                  f"thr={cmd.throttle:+.2f} ste={cmd.steering:+.2f} hiz={logic.speed_scale():.1f} | "
                  f"Teensy:{'OK' if link_ok else 'YOK'} st={tm.get('state', '-')} "
                  f"L={tm.get('spdL', '-')} R={tm.get('spdR', '-')} yaw={tm.get('yaw', '-')} "
                  f"fork={tm.get('fork', '-')}/{tm.get('forkSt', '-')} "
                  f"bat={tm.get('bat', '-')}V motor={tm.get('motL', '-')}/{tm.get('motR', '-')}",
                  flush=True)
            last_print = t0

        time.sleep(max(0.0, period - (time.monotonic() - t0)))


if __name__ == '__main__':
    main()
