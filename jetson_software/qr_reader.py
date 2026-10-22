#!/usr/bin/env python3
# ============================================================================
#  AVTECH AGV — QR Kod Okuyucu + Pozisyon Hesaplama
#  Şartname 3.1.1/5: "QR kod okuma ve QR kodun kameraya göre pozisyonunu
#                      hesaplayabilmelidir."
#
#  Kullanım:
#    qr = QRReader(qr_size_m=0.10)  # QR kod fiziksel boyutu 10cm
#    result = qr.process(frame)     # Kamera frame'i işle
#    if result['detected']:
#        print(result['data'])      # QR içeriği
#        print(result['tvec'])      # Kameraya göre 3D pozisyon
# ============================================================================

import cv2
import numpy as np
import time
import math
import json
import os

# pyzbar import — kurulu değilse OpenCV QR detector'a düş
try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False
    print("[QR] pyzbar bulunamadı, OpenCV QRCodeDetector kullanılacak")
    print("[QR] Kurulum: pip install pyzbar")


class QRReader:
    """QR kod algılama + 6-DoF kamera-göreceli pozisyon tahmini."""

    def __init__(self, qr_size_m=0.10, camera_matrix=None, dist_coeffs=None):
        """
        Args:
            qr_size_m: QR kodun fiziksel kenar uzunluğu (metre)
            camera_matrix: 3x3 kamera intrinsic matrisi (None ise varsayılan)
            dist_coeffs: Distorsiyon katsayıları (None ise sıfır distorsiyon)
        """
        self.qr_size = qr_size_m

        # Kalibrasyon dosyasından okumayı dene
        calib_file = 'camera_calib.json'
        loaded_calib = False
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    calib_data = json.load(f)
                    self.camera_matrix = np.array(calib_data['camera_matrix'], dtype=np.float64)
                    self.dist_coeffs = np.array(calib_data['dist_coeffs'], dtype=np.float64)
                    loaded_calib = True
                    print(f"[QR] Kamera kalibrasyonu {calib_file} dosyasından yüklendi.")
            except Exception as e:
                print(f"[QR] Kalibrasyon dosyası okunamadı: {e}")

        # Kamera kalibrasyon — varsayılan Logitech C922 @ 640x480 (Eğer dosya veya parametre yoksa)
        if not loaded_calib:
            if camera_matrix is not None:
                self.camera_matrix = np.array(camera_matrix, dtype=np.float64)
            else:
                # Logitech C922 yaklaşık intrinsic parametreleri (640x480)
                fx = 600.0  # Odak uzaklığı (piksel)
                fy = 600.0
                cx = 320.0  # Optik merkez
                cy = 240.0
                self.camera_matrix = np.array([
                    [fx, 0,  cx],
                    [0,  fy, cy],
                    [0,  0,  1 ]
                ], dtype=np.float64)

            if dist_coeffs is not None:
                self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
            else:
                self.dist_coeffs = np.zeros(5, dtype=np.float64)

        # QR kodun 3D model noktaları (obje koordinat sistemi)
        # Merkez orijinli, XY düzleminde, Z=0
        half = qr_size_m / 2.0
        self.object_points = np.array([
            [-half, -half, 0],  # Sol-üst
            [ half, -half, 0],  # Sağ-üst
            [ half,  half, 0],  # Sağ-alt
            [-half,  half, 0],  # Sol-alt
        ], dtype=np.float64)

        # OpenCV QR detector (fallback)
        self.cv_detector = cv2.QRCodeDetector()

        # Son sonuç cache
        self.last_result = self._empty_result()

        print(f"[QR] Reader initialized (QR size: {qr_size_m*100:.0f}cm)")
        print(f"[QR] Backend: {'pyzbar' if PYZBAR_AVAILABLE else 'OpenCV'}")

    def _empty_result(self):
        return {
            'detected': False,
            'data': '',
            'corners': None,
            'center_px': (0, 0),
            'tvec': (0, 0, 0),       # Translation: (x, y, z) metre
            'rvec': (0, 0, 0),       # Rotation: (rx, ry, rz) radian
            'distance_m': 0.0,       # QR koda mesafe
            'angle_deg': 0.0,        # QR kodun açısal sapması (derece)
            'offset_x_cm': 0.0,      # Yatay sapma (cm)
            'timestamp': 0.0,
        }

    def process(self, frame):
        """Kamera frame'inde QR kod ara ve pozisyon hesapla.

        Args:
            frame: BGR OpenCV frame (numpy array)

        Returns:
            dict: QR kod verisi, pozisyon, mesafe, açı
        """
        if frame is None:
            return self._empty_result()

        result = self._empty_result()
        result['timestamp'] = time.time()

        # Gri tonlama
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        qr_data = None
        qr_corners = None

        # ── QR Kod Tespit ──
        if PYZBAR_AVAILABLE:
            from pyzbar.pyzbar import ZBarSymbol
            decoded = pyzbar.decode(gray, symbols=[ZBarSymbol.QRCODE])
            if decoded:
                qr = decoded[0]  # İlk QR kod
                qr_data = qr.data.decode('utf-8', errors='ignore')
                # pyzbar polygon → numpy array (4 köşe)
                pts = qr.polygon
                if len(pts) == 4:
                    qr_corners = np.array(
                        [[p.x, p.y] for p in pts],
                        dtype=np.float64
                    )
        else:
            # OpenCV fallback
            data, points, _ = self.cv_detector.detectAndDecode(gray)
            if data and points is not None:
                qr_data = data
                qr_corners = points[0].astype(np.float64)

        if qr_data is None or qr_corners is None or len(qr_corners) < 4:
            self.last_result = result
            return result

        # ── QR Kod Bulundu ──
        result['detected'] = True
        result['data'] = qr_data
        result['corners'] = qr_corners.tolist()

        # Merkez noktası
        cx = np.mean(qr_corners[:, 0])
        cy = np.mean(qr_corners[:, 1])
        result['center_px'] = (float(cx), float(cy))

        # ── 6-DoF Pozisyon Tahmini (solvePnP) ──
        try:
            image_points = qr_corners.astype(np.float64)

            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE  # Düzlemsel hedefler için optimize
            )

            if success:
                # Translation vector: (x, y, z) metre cinsinden
                tx, ty, tz = tvec.flatten()
                result['tvec'] = (float(tx), float(ty), float(tz))
                result['rvec'] = tuple(rvec.flatten().tolist())

                # Mesafe: Öklid normu
                distance = math.sqrt(tx*tx + ty*ty + tz*tz)
                result['distance_m'] = float(distance)

                # Açısal sapma: atan2(x, z) → kameranın optik ekseninden sapma
                angle = math.degrees(math.atan2(tx, tz))
                result['angle_deg'] = float(angle)

                # Yatay sapma cm cinsinden
                result['offset_x_cm'] = float(tx * 100.0)

        except Exception as e:
            print(f"[QR] solvePnP hatası: {e}")

        self.last_result = result
        return result


    def draw_overlay(self, frame, result=None):
        """QR kod algılama sonucunu frame üzerine çiz.

        Args:
            frame: BGR frame
            result: process() sonucu (None ise son sonucu kullan)

        Returns:
            frame: Overlay eklenmiş frame
        """
        if result is None:
            result = self.last_result

        if not result['detected'] or result['corners'] is None:
            return frame

        corners = np.array(result['corners'], dtype=np.int32)

        # QR kod çerçevesi
        cv2.polylines(frame, [corners], True, (0, 255, 0), 3)

        # Merkez noktası
        cx, cy = result['center_px']
        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        # Bilgi metni
        y_offset = max(0, int(corners[0][1]) - 50)

        # QR data
        cv2.putText(frame, f"QR: {result['data']}",
                    (int(corners[0][0]), y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Mesafe
        cv2.putText(frame, f"Dist: {result['distance_m']:.2f}m",
                    (int(corners[0][0]), y_offset + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Açı
        cv2.putText(frame, f"Angle: {result['angle_deg']:.1f}deg",
                    (int(corners[0][0]), y_offset + 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Offset
        cv2.putText(frame, f"Offset: {result['offset_x_cm']:.1f}cm",
                    (int(corners[0][0]), y_offset + 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

        return frame

    def get_last_result(self):
        """Son QR okuma sonucunu döndür."""
        return self.last_result

    def set_camera_calibration(self, camera_matrix, dist_coeffs):
        """Kamera kalibrasyon parametrelerini güncelle."""
        self.camera_matrix = np.array(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
        print("[QR] Kamera kalibrasyonu güncellendi")


# --- Standalone Test ---

if __name__ == '__main__':
    import sys

    qr = QRReader(qr_size_m=0.10)

    # Kamera ile test
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Kamera açılamadı!")
        sys.exit(1)

    print("QR kod okuyucu çalışıyor. 'q' ile çıkış...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result = qr.process(frame)
        frame = qr.draw_overlay(frame, result)

        if result['detected']:
            print(f"  QR: '{result['data']}' | "
                  f"Mesafe: {result['distance_m']:.2f}m | "
                  f"Açı: {result['angle_deg']:.1f}° | "
                  f"Offset: {result['offset_x_cm']:.1f}cm")

        cv2.imshow("AVTECH AGV - QR Reader", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
