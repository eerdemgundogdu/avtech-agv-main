#!/usr/bin/env python3
#  Gereksinimler:
#    pip3 install pyserial
#
#  Bu script Teensy'deki comm_test.ino firmware'i ile haberleşmeyi test eder.
# ============================================================================

import serial
import serial.tools.list_ports
import json
import time
import sys
import threading

# --- Ayarlar ---

BAUD_RATE = 115200
TIMEOUT   = 2.0      # Serial okuma timeout (saniye)

# --- Teensy Port Bulma ---

def find_teensy_port():
    """USB üzerinden bağlı Teensy'yi otomatik bul."""
    ports = serial.tools.list_ports.comports()
    teensy_ports = []

    for port in ports:
        desc = (port.description or "").lower()
        mfr  = (port.manufacturer or "").lower()

        # Teensy USB tanımlayıcıları
        if any(key in desc for key in ["teensy", "usb serial"]):
            teensy_ports.append(port)
        elif any(key in mfr for key in ["teensy", "pjrc"]):
            teensy_ports.append(port)
        elif port.vid == 0x16C0:  # PJRC Vendor ID
            teensy_ports.append(port)

    # ACM veya ttyUSB portlarını da kontrol et
    if not teensy_ports:
        for port in ports:
            if "ACM" in port.device or "ttyUSB" in port.device:
                teensy_ports.append(port)

    return teensy_ports


def list_all_ports():
    """Tüm seri portları listele."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  Hiçbir seri port bulunamadı!")
        return

    print(f"\n  {'Port':<20} {'Açıklama':<30} {'Üretici':<20} {'VID:PID'}")
    print(f"  {'─'*20} {'─'*30} {'─'*20} {'─'*15}")
    for p in ports:
        vid_pid = f"{p.vid:04X}:{p.pid:04X}" if p.vid else "N/A"
        print(f"  {p.device:<20} {(p.description or 'N/A'):<30} {(p.manufacturer or 'N/A'):<20} {vid_pid}")

# --- Serial Haberleşme ---

class TeensyComm:
    def __init__(self, port, baud=BAUD_RATE):
        self.port = port
        self.baud = baud
        self.ser = None
        self.listener_thread = None
        self.running = False
        self.rx_buffer = []
        self.lock = threading.Lock()

    def connect(self):
        """Seri porta bağlan."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=TIMEOUT,
                write_timeout=TIMEOUT
            )
            # Teensy'nin reset olmasını bekle
            time.sleep(2.0)
            # Buffer'ı temizle
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return True
        except serial.SerialException as e:
            print(f"  Bağlantı hatası: {e}")
            return False

    def disconnect(self):
        """Bağlantıyı kes."""
        self.stop_listener()
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_json(self, data: dict) -> bool:
        """JSON komut gönder."""
        if not self.ser or not self.ser.is_open:
            print("  Seri port bağlı değil!")
            return False

        try:
            msg = json.dumps(data, separators=(',', ':')) + '\n'
            self.ser.write(msg.encode('utf-8'))
            self.ser.flush()
            return True
        except serial.SerialException as e:
            print(f"  Gönderim hatası: {e}")
            return False

    def read_line(self, timeout=TIMEOUT):
        """Bir satır oku."""
        if not self.ser or not self.ser.is_open:
            return None

        try:
            old_timeout = self.ser.timeout
            self.ser.timeout = timeout
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
            self.ser.timeout = old_timeout
            return line if line else None
        except serial.SerialException:
            return None

    def read_all_available(self, timeout=0.5):
        """Mevcut tüm satırları oku."""
        lines = []
        end_time = time.time() + timeout
        while time.time() < end_time:
            line = self.read_line(timeout=0.1)
            if line:
                lines.append(line)
            else:
                break
        return lines

    def send_and_receive(self, data: dict, timeout=TIMEOUT):
        """Komut gönder ve yanıtı al."""
        # Önce buffer'ı temizle
        self.ser.reset_input_buffer()

        if not self.send_json(data):
            return None

        # Yanıtları oku
        responses = []
        end_time = time.time() + timeout
        while time.time() < end_time:
            line = self.read_line(timeout=0.3)
            if line:
                responses.append(line)
                # JSON yanıtı mı kontrol et
                try:
                    parsed = json.loads(line)
                    if "resp" in parsed:
                        break  # Ana yanıtı bulduk
                except json.JSONDecodeError:
                    pass

        return responses

    # ── Background Listener ──

    def start_listener(self):
        """Arka planda sürekli dinle."""
        self.running = True
        self.listener_thread = threading.Thread(target=self._listener_loop, daemon=True)
        self.listener_thread.start()

    def stop_listener(self):
        """Dinleyiciyi durdur."""
        self.running = False
        if self.listener_thread:
            self.listener_thread.join(timeout=2.0)

    def _listener_loop(self):
        while self.running:
            line = self.read_line(timeout=0.5)
            if line:
                timestamp = time.strftime("%H:%M:%S")

                # JSON formatını güzelleştir
                try:
                    parsed = json.loads(line)
                    if "heartbeat" in parsed:
                        print(f"\r   [{timestamp}] HB: uptime={parsed.get('uptime_ms',0)}ms rx={parsed.get('rx',0)} tx={parsed.get('tx',0)}")
                    else:
                        print(f"\r   [{timestamp}] {json.dumps(parsed, indent=2, ensure_ascii=False)}")
                except json.JSONDecodeError:
                    print(f"\r   [{timestamp}] {line}")

                print("  > ", end="", flush=True)

# --- Test Fonksiyonları ---

def test_ping(comm: TeensyComm):
    """Ping-Pong testi."""
    print("\n  ── Ping Testi ──")

    t0 = time.time()
    responses = comm.send_and_receive({"cmd": "ping"})
    elapsed_ms = (time.time() - t0) * 1000

    if responses:
        for r in responses:
            try:
                parsed = json.loads(r)
                if parsed.get("resp") == "pong":
                    print(f"  PONG alındı! RTT: {elapsed_ms:.1f}ms, Teensy uptime: {parsed.get('uptime_ms',0)}ms")
                    return True
            except json.JSONDecodeError:
                print(f"  {r}")

    print(f"  Yanıt alınamadı ({elapsed_ms:.1f}ms)")
    return False


def test_echo(comm: TeensyComm):
    """Echo testi."""
    print("\n  ── Echo Testi ──")

    test_messages = [
        "merhaba",
        "AVTECH AGV 2026",
        "test_123_abc",
        "türkçe karakterler: çğıöşü",
    ]

    success = 0
    for msg in test_messages:
        responses = comm.send_and_receive({"cmd": "echo", "msg": msg})
        if responses:
            for r in responses:
                try:
                    parsed = json.loads(r)
                    if parsed.get("resp") == "echo" and parsed.get("msg") == msg:
                        print(f"  Echo OK: \"{msg}\"")
                        success += 1
                        break
                except json.JSONDecodeError:
                    pass
        else:
            print(f"  Echo BAŞARISIZ: \"{msg}\"")

        time.sleep(0.1)

    print(f"\n  Sonuç: {success}/{len(test_messages)} başarılı")
    return success == len(test_messages)


def test_latency(comm: TeensyComm, count=20):
    """Gecikme (latency) testi."""
    print(f"\n  ── Gecikme Testi ({count} ping) ──")

    latencies = []
    for i in range(count):
        comm.ser.reset_input_buffer()
        t0 = time.time()
        comm.send_json({"cmd": "ping"})

        # Yanıt bekle
        end_time = time.time() + 2.0
        got_pong = False
        while time.time() < end_time:
            line = comm.read_line(timeout=0.5)
            if line:
                try:
                    parsed = json.loads(line)
                    if parsed.get("resp") == "pong":
                        elapsed_ms = (time.time() - t0) * 1000
                        latencies.append(elapsed_ms)
                        got_pong = True
                        break
                except json.JSONDecodeError:
                    pass

        if not got_pong:
            print(f"  Ping #{i+1}: yanıt yok")

        time.sleep(0.05)

    if latencies:
        avg = sum(latencies) / len(latencies)
        mn  = min(latencies)
        mx  = max(latencies)
        print(f"  Başarılı: {len(latencies)}/{count}")
        print(f"  Min:      {mn:.1f} ms")
        print(f"  Ort:      {avg:.1f} ms")
        print(f"  Max:      {mx:.1f} ms")
    else:
        print("  Hiçbir yanıt alınamadı!")

    return len(latencies) == count


def test_stress(comm: TeensyComm, count=100):
    """Throughput stres testi."""
    print(f"\n  ── Stres Testi ({count} mesaj) ──")

    comm.ser.reset_input_buffer()
    comm.send_json({"cmd": "stress", "count": count})

    received = 0
    end_time = time.time() + 10.0
    summary = None

    while time.time() < end_time:
        line = comm.read_line(timeout=0.5)
        if line:
            try:
                parsed = json.loads(line)
                if parsed.get("resp") == "stress":
                    received += 1
                elif parsed.get("resp") == "stress_done":
                    summary = parsed
                    break
            except json.JSONDecodeError:
                pass

    print(f"  Alınan mesaj: {received}/{count}")
    if summary:
        print(f"  Teensy tarafı: {summary.get('elapsed_us', 0)} µs")
        print(f"  Throughput:    {summary.get('msg_per_sec', 0):.0f} msg/s")

    return received == count


def test_led(comm: TeensyComm):
    """LED kontrol testi."""
    print("\n  ── LED Kontrol Testi ──")

    # LED aç
    responses = comm.send_and_receive({"cmd": "led_on"})
    if responses:
        print("  LED ON komutu gönderildi")
    time.sleep(1.0)

    # LED kapat
    responses = comm.send_and_receive({"cmd": "led_off"})
    if responses:
        print("  LED OFF komutu gönderildi")
    time.sleep(0.5)

    # LED blink
    responses = comm.send_and_receive({"cmd": "led_blink", "ms": 200})
    if responses:
        print("  LED BLINK komutu gönderildi (200ms)")
    time.sleep(2.0)

    # LED kapat
    comm.send_json({"cmd": "led_off"})
    print("  LED kapatıldı")


def test_status(comm: TeensyComm):
    """Durum sorgulama testi."""
    print("\n  ── Durum Sorgusu ──")

    responses = comm.send_and_receive({"cmd": "status"})
    if responses:
        for r in responses:
            try:
                parsed = json.loads(r)
                if parsed.get("resp") == "status":
                    print(f"  Uptime:     {parsed.get('uptime_ms', 0)} ms")
                    print(f"  RX mesaj:   {parsed.get('msg_rx', 0)}")
                    print(f"  TX mesaj:   {parsed.get('msg_tx', 0)}")
                    print(f"  Hatalar:    {parsed.get('errors', 0)}")
                    print(f"  LED:        {parsed.get('led', '?')}")
                    print(f"  Heartbeat:  {parsed.get('heartbeat', '?')}")
                    return True
            except json.JSONDecodeError:
                print(f"  {r}")

    print("  Durum yanıtı alınamadı")
    return False


def run_all_tests(comm: TeensyComm):
    """Tüm testleri sırayla çalıştır."""
    print("\n" + "="*50)
    print("  AVTECH AGV — Tam Haberleşme Testi")
    print("="*50)

    results = {}

    # Heartbeat'i kapat (test sırasında karıştırmasın)
    comm.send_and_receive({"cmd": "heartbeat_off"})
    time.sleep(0.3)
    comm.ser.reset_input_buffer()

    results["Ping"]     = test_ping(comm)
    results["Echo"]     = test_echo(comm)
    results["Latency"]  = test_latency(comm)
    results["Stress"]   = test_stress(comm)
    results["LED"]      = True  # Görsel test
    test_led(comm)
    results["Status"]   = test_status(comm)

    # Heartbeat'i tekrar aç
    comm.send_json({"cmd": "heartbeat_on"})

    # Sonuç özeti
    print("\n" + "="*50)
    print("  TEST SONUÇLARI")
    print("="*50)
    for name, passed in results.items():
        icon = "" if passed else ""
        print(f"  {icon} {name}")

    total   = len(results)
    passed  = sum(1 for v in results.values() if v)
    print(f"\n  Toplam: {passed}/{total} başarılı")
    print("="*50)

# --- İnteraktif Menü ---

def interactive_menu(comm: TeensyComm):
    """İnteraktif komut menüsü."""

    print("\n" + "="*50)
    print("  AVTECH AGV — İnteraktif Haberleşme Testi")
    print("="*50)
    print()
    print("  Komutlar:")
    print("  1  Ping testi")
    print("  2  Echo testi")
    print("  3  Gecikme testi (20 ping)")
    print("  4  Stres testi (100 mesaj)")
    print("  5  LED kontrol")
    print("  6  Durum sorgula")
    print("  7  Tüm testleri çalıştır")
    print("  8  Sürekli dinle (heartbeat izle)")
    print("  9  Manuel JSON gönder")
    print("  0  Çıkış")
    print()

    while True:
        try:
            choice = input("  > Seçiminiz: ").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if choice == "1":
            test_ping(comm)
        elif choice == "2":
            test_echo(comm)
        elif choice == "3":
            test_latency(comm)
        elif choice == "4":
            test_stress(comm)
        elif choice == "5":
            print("  a) LED aç  b) LED kapat  c) LED blink")
            sub = input("    > ").strip().lower()
            if sub == "a":
                comm.send_and_receive({"cmd": "led_on"})
                print("  LED açıldı")
            elif sub == "b":
                comm.send_and_receive({"cmd": "led_off"})
                print("  LED kapatıldı")
            elif sub == "c":
                comm.send_and_receive({"cmd": "led_blink", "ms": 300})
                print("  LED blink başladı")
        elif choice == "6":
            test_status(comm)
        elif choice == "7":
            run_all_tests(comm)
        elif choice == "8":
            print("  Sürekli dinleme modu (Ctrl+C ile çık)...")
            comm.start_listener()
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                comm.stop_listener()
                print("\n    Dinleme durduruldu.")
        elif choice == "9":
            raw = input("    JSON girin: ").strip()
            try:
                data = json.loads(raw)
                responses = comm.send_and_receive(data)
                if responses:
                    for r in responses:
                        try:
                            parsed = json.loads(r)
                            print(f"  {json.dumps(parsed, indent=2, ensure_ascii=False)}")
                        except json.JSONDecodeError:
                            print(f"  {r}")
            except json.JSONDecodeError:
                print("  Geçersiz JSON!")
        elif choice == "0":
            break
        else:
            print("  Geçersiz seçim!")

        print()

# --- Main ---

def main():
    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║  AVTECH AGV — Jetson ↔ Teensy Haberleşme Testi  ║")
    print("║  JetPack 6.2.3 / Ubuntu 22.04 / ROS2 Humble    ║")
    print("╚══════════════════════════════════════════════════╝")
    print()

    # Port belirleme
    port = None

    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"  Manuel port: {port}")
    else:
        print("  Teensy aranıyor...")
        teensy_ports = find_teensy_port()

        if teensy_ports:
            port = teensy_ports[0].device
            print(f"  Teensy bulundu: {port}")
            if len(teensy_ports) > 1:
                print(f"  ℹ {len(teensy_ports)} port bulundu, ilki seçildi.")
        else:
            print("  Teensy bulunamadı!")
            print("\n  Mevcut seri portlar:")
            list_all_ports()
            print("\n  Kullanım: python3 comm_test_jetson.py /dev/ttyACM0")
            sys.exit(1)

    # Tüm portları göster
    print("\n  Tüm seri portlar:")
    list_all_ports()

    # Bağlan
    print(f"\n  {port} portuna bağlanılıyor ({BAUD_RATE} baud)...")
    comm = TeensyComm(port)

    if not comm.connect():
        sys.exit(1)

    print("  Bağlantı başarılı!")

    # İlk verileri oku (banner vs.)
    print("\n  Teensy'den gelen başlangıç mesajları:")
    initial = comm.read_all_available(timeout=2.0)
    for line in initial:
        print(f"  {line}")

    try:
        # Hızlı bir ping at
        print()
        if test_ping(comm):
            print("\n   Haberleşme doğrulandı! ")
        else:
            print("\n   Ping yanıtı alınamadı, ancak bağlantı açık.")

        # İnteraktif menü
        interactive_menu(comm)

    except KeyboardInterrupt:
        print("\n\n  Ctrl+C — Çıkılıyor...")
    finally:
        comm.disconnect()
        print("  Bağlantı kapatıldı.")
        print()


if __name__ == "__main__":
    main()
