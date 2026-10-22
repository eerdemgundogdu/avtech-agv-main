#!/usr/bin/env python3
# ============================================================================
# AVTECH AGV - QR KOD & GOREV (AL/BIRAK) TEST KODU
#
# Bu kod, Jetson uzerinde kamerayi (Logitech C922) acarak QR kodlari okur.
# Okunan QR koda gore "Yuk Al" veya "Yuk Birak" komutlarinin dogru 
# uretilip uretilmedigini test eder.
#
# KULLANIM:
# python3 jetson_qr_action_test.py
# (Kameraya bir QR kod okutun. Iceriginde "AL" veya "BIRAK" kelimeleri gecmeli)
# ============================================================================

import time
import sys
import os
import cv2

# Ayni dizindeki camera_node.py icerisindeki CameraNode sinifini import ediyoruz
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from camera_node import CameraNode
except ImportError:
    print("[HATA] camera_node.py bulunamadi! Lutfen bu dosyayi jetson_software klasorunde calistirin.")
    sys.exit(1)

def simulate_teensy_command(command, action):
    """
    Bu fonksiyon asil yazilimdaki seri port haberlesmesini simule eder.
    Asil yazilimda bu komut Teensy'ye (UART uzerinden) gonderilir ve 
    Teensy step motoru (asansoru) harekete gecirir.
    """
    print("="*50)
    print(f" [TEENSY'YE KOMUT GONDERILDI] Modul: {command} -> Aksiyon: {action}")
    if action == "OPEN":
        print("   --> Beklenen Donanim Tepkisi: Asansor (Step Motor) YUKARI (Almak/Birakmak icin hazirlik)")
    elif action == "CLOSE":
        print("   --> Beklenen Donanim Tepkisi: Asansor (Step Motor) ASAGI (Yuku Kavrama/Birakma)")
    print("="*50)

def run_qr_test():
    print("=====================================")
    print("QR KOD ve GOREV TETIKLEME TESTI")
    print("=====================================")
    print("[BILGI] Kamera baslatiliyor (Lutfen bekleyin)...")

    cam = CameraNode(camera_index=0, width=640, height=480)

    if not cam.start():
        print("[HATA] Kamera acilamadi. USB baglantisini kontrol edin.")
        return

    print("[BASARILI] Kamera acildi! Lutfen kameraya bir QR kod gosterin.")
    print("Cikmak icin terminalde Ctrl+C yapin (veya pencere uzerinde 'q' tusuna basin).")

    # Ayni kodu pes pese tetiklememek icin basit bir debounce/hatirlatici
    last_detected_data = ""
    last_detect_time = 0
    debounce_delay = 5.0 # Ayni kodu 5 saniye icinde tekrar tetikleme

    try:
        while True:
            # Sadece ekranda gormek istersek (Opsiyonel, Jetson'da GUI varsa acilir)
            frame = cam.get_frame()
            if frame is not None:
                cv2.imshow("QR Test Ekrani (Cikmak icin 'q')", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            qr = cam.get_qr_result()
            if qr.get('detected'):
                current_data = qr['data'].upper()
                now = time.time()

                # Ayni kodu 5 saniyede bir isleme (Robot ayni noktada beklerken spamlari engellemek icin)
                if current_data != last_detected_data or (now - last_detect_time) > debounce_delay:
                    print(f"\n[QR TESPIT] Icerik: '{current_data}', Uzaklik: {qr.get('distance_m', 0.0):.2f}m")

                    if "AL" in current_data or "PICK" in current_data:
                        print(">> QR Kod analizi: YUK ALMA GOREVI")
                        simulate_teensy_command("GRIPPER", "CLOSE")

                    elif "BIRAK" in current_data or "DROP" in current_data or "DUMP" in current_data:
                        print(">> QR Kod analizi: YUK BIRAKMA GOREVI")
                        simulate_teensy_command("GRIPPER", "OPEN")

                    else:
                        print(">> QR Kod analizi: Gecersiz gorev kelimesi. (Icerikte 'AL' veya 'BIRAK' gecmiyor)")

                    last_detected_data = current_data
                    last_detect_time = now

            time.sleep(0.05) # CPU'yu yormamak icin 20Hz dongu

    except KeyboardInterrupt:
        print("\nTest sonlandiriliyor...")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print("Kamera basariyla kapatildi.")

if __name__ == "__main__":
    run_qr_test()
