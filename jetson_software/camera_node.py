#!/usr/bin/env python3
# ============================================================================
#  AVTECH AGV — Kamera Node (Logitech C922)
#  Şartname 3.1.1/4: "Renkli şeritleri görüntü işleme ile algılayıp takip"
#
#  Özellikler:
#  - Renkli çizgi algılama (kırmızı, mavi, yeşil, sarı)
#  - Bird's-eye view perspektif düzeltme
#  - Piksel sapmasını cm cinsinden hesaplama
#  - ArUco marker algılama
#  - QR kod okuma (qr_reader.py entegrasyonu)
#  - MJPEG stream (Flask endpoint)
# ============================================================================

import cv2
import time
import threading
import math
import numpy as np

from qr_reader import QRReader


class ColorLineDetector:
    """Renkli çizgi/şerit algılama — HSV tabanlı.

    Şartname: "Yük alma/bırakma noktalarının 1.5m öncesindeki renkli şeritleri
    görüntü işleme yöntemleri ile algılayıp takip edebilmelidir."
    """

    # HSV renk aralıkları (OpenCV H: 0-179, S: 0-255, V: 0-255)
    COLOR_RANGES = {
        'red_lower':  {'low': np.array([0, 100, 80]),   'high': np.array([10, 255, 255])},
        'red_upper':  {'low': np.array([160, 100, 80]),  'high': np.array([179, 255, 255])},
        'blue':       {'low': np.array([100, 80, 60]),   'high': np.array([130, 255, 255])},
        'green':      {'low': np.array([40, 60, 50]),    'high': np.array([80, 255, 255])},
        'yellow':     {'low': np.array([20, 80, 80]),    'high': np.array([40, 255, 255])},
        'black':      {'low': np.array([0, 0, 0]),       'high': np.array([179, 255, 45])},
    }

    # Çizgi tespit renk isimleri ve overlay renkleri (BGR)
    DISPLAY_COLORS = {
        'red':    (0, 0, 255),
        'blue':   (255, 100, 0),
        'green':  (0, 200, 0),
        'yellow': (0, 220, 255),
        'black':  (128, 128, 128),
    }

    def __init__(self, frame_width=640, frame_height=480):
        self.width = frame_width
        self.height = frame_height

        # ROI (Region of Interest) — çizgi alt yarıda olur
        self.roi_top = int(frame_height * 0.55)
        self.roi_bottom = frame_height

        # Morfoloji kernel
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # Bird's-eye view perspektif matrisi
        self._compute_perspective()

        # Son sonuç
        self.last_result = {
            'detected': False,
            'color': '',
            'offset_cm': 0.0,
            'offset_px': 0,
            'confidence': 0.0,
            'contour_area': 0,
        }

    def _compute_perspective(self):
        """Bird's-eye view dönüşüm matrisi hesapla."""
        # Kaynak noktalar (trapez — yolun perspektif görünümü)
        w, h = self.width, self.height
        src = np.float32([
            [w * 0.1, h * 0.95],    # Sol-alt
            [w * 0.9, h * 0.95],    # Sağ-alt
            [w * 0.65, h * 0.55],   # Sağ-üst
            [w * 0.35, h * 0.55],   # Sol-üst
        ])
        # Hedef noktalar (dikdörtgen — kuş bakışı)
        dst = np.float32([
            [w * 0.2, h],
            [w * 0.8, h],
            [w * 0.8, 0],
            [w * 0.2, 0],
        ])
        self.perspective_matrix = cv2.getPerspectiveTransform(src, dst)
        self.perspective_matrix_inv = cv2.getPerspectiveTransform(dst, src)

    def detect(self, frame):
        """Renkli çizgi algıla.

        Args:
            frame: BGR kamera frame'i

        Returns:
            dict: {detected, color, offset_cm, offset_px, confidence, contour_area}
        """
        if frame is None:
            return self.last_result

        # ROI kırp
        roi = frame[self.roi_top:self.roi_bottom, :].copy()

        # Gaussian blur (gürültü azaltma)
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)

        # HSV'ye çevir
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        best_result = None
        best_area = 0

        # Her renk için tarama
        for color_name in ['red', 'blue', 'green', 'yellow', 'black']:
            mask = self._create_mask(hsv, color_name)

            # Morfoloji: gürültü temizle
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

            # Kontur bul
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            # En büyük konturu bul
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            # Minimum alan eşiği (gürültü filtresi)
            min_area = (self.width * (self.roi_bottom - self.roi_top)) * 0.005
            if area < min_area:
                continue

            if area > best_area:
                best_area = area

                # Kontur merkezi
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                else:
                    cx = self.width // 2

                # Piksel sapması (merkez = 0)
                offset_px = cx - self.width // 2

                # Piksel → cm dönüşümü (Trigonometrik yaklaşım)
                # Logitech C922 varsayılan odak uzaklığı (fx) ~600 piksel
                # Çizginin kameradan yaklaşık 150cm (1.5m) ileride olduğunu varsayıyoruz
                target_distance_cm = 150.0
                focal_length_px = 600.0

                # Merkezden olan piksel sapmasını açıya çevir
                angle_rad = math.atan2(offset_px, focal_length_px)

                # Açıdan fiziksel sapmayı (cm) hesapla
                offset_cm = target_distance_cm * math.tan(angle_rad)

                # Güvenilirlik: alan / ROI alanı
                roi_area = self.width * (self.roi_bottom - self.roi_top)
                confidence = min(area / roi_area * 10, 1.0)

                best_result = {
                    'detected': True,
                    'color': color_name,
                    'offset_cm': round(offset_cm, 2),
                    'offset_px': offset_px,
                    'confidence': round(confidence, 2),
                    'contour_area': int(area),
                }

        if best_result is None:
            best_result = {
                'detected': False,
                'color': '',
                'offset_cm': 0.0,
                'offset_px': 0,
                'confidence': 0.0,
                'contour_area': 0,
            }

        self.last_result = best_result
        return best_result

    def _create_mask(self, hsv, color_name):
        """Belirli renk için HSV maskesi oluştur."""
        if color_name == 'red':
            # Kırmızı H=0 civarında wrap yapar → 2 aralık birleştir
            mask1 = cv2.inRange(hsv,
                                self.COLOR_RANGES['red_lower']['low'],
                                self.COLOR_RANGES['red_lower']['high'])
            mask2 = cv2.inRange(hsv,
                                self.COLOR_RANGES['red_upper']['low'],
                                self.COLOR_RANGES['red_upper']['high'])
            return mask1 | mask2
        else:
            ranges = self.COLOR_RANGES[color_name]
            return cv2.inRange(hsv, ranges['low'], ranges['high'])

    def draw_overlay(self, frame, result=None):
        """Çizgi algılama sonucunu frame üzerine çiz."""
        if result is None:
            result = self.last_result

        # ROI çerçevesi
        cv2.line(frame, (0, self.roi_top), (self.width, self.roi_top),
                 (100, 100, 100), 1)

        if not result['detected']:
            cv2.putText(frame, "LINE: NOT DETECTED", (10, self.roi_top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 1)
            return frame

        color = self.DISPLAY_COLORS.get(result['color'], (255, 255, 255))

        # Merkez çizgi
        center_x = self.width // 2 + result['offset_px']
        cv2.line(frame, (center_x, self.roi_top), (center_x, self.roi_bottom),
                 color, 3)

        # Referans merkez çizgisi
        cv2.line(frame, (self.width // 2, self.roi_top),
                 (self.width // 2, self.roi_bottom), (200, 200, 200), 1)

        # Bilgi metni
        info = (f"LINE: {result['color'].upper()} | "
                f"Offset: {result['offset_cm']:.1f}cm | "
                f"Conf: {result['confidence']:.0%}")
        cv2.putText(frame, info, (10, self.roi_top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        return frame


class CameraNode:
    """Logitech C922 kamera yönetici.

    Görevler:
    1. MJPEG stream (GUI için)
    2. Renkli çizgi algılama
    3. QR kod okuma + pozisyon
    4. ArUco marker algılama
    """

    def __init__(self, camera_index=0, width=640, height=480, fps=30):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.running = False
        self.frame = None
        self.frame_lock = threading.Lock()

        # Alt modüller
        self.line_detector = ColorLineDetector(width, height)
        self.qr_reader = QRReader(qr_size_m=0.10)

        # ArUco detector
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.aruco_available = True
        except Exception:
            self.aruco_available = False
            print("[CAMERA] ArUco kullanılamıyor (opencv-contrib gerekli)")

        # Sonuçlar
        self.marker_data = {'ids': [], 'count': 0}
        self.line_result = {}
        self.qr_result = {}
        self._process_interval = 0.05  # 20 Hz işleme
        self._last_process_time = 0

    def start(self):
        """Kamera yakalamayı başlat."""
        try:
            # Jetson'da GStreamer, yoksa V4L2, yoksa default
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.camera_index)

            if not self.cap.isOpened():
                print(f"[CAMERA] Kamera {self.camera_index} açılamadı!")
                return False

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Sabit fokus

            self.running = True

            # Yakalama thread'i
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()

            actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            print(f"[CAMERA] C922 started: {int(actual_w)}x{int(actual_h)} @ {actual_fps:.0f}FPS")
            return True

        except Exception as e:
            print(f"[CAMERA] Başlatma hatası: {e}")
            return False

    def stop(self):
        """Kamerayı durdur."""
        self.running = False
        if self.cap:
            self.cap.release()

    def _capture_loop(self):
        """Arka planda sürekli frame yakala ve işle."""
        while self.running:
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.1)
                continue

            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            now = time.time()
            process_frame = (now - self._last_process_time) >= self._process_interval

            if process_frame:
                self._last_process_time = now

                # ── 1) Renkli çizgi algılama ──
                self.line_result = self.line_detector.detect(frame)

                # ── 2) QR kod okuma ──
                self.qr_result = self.qr_reader.process(frame)

                # ── 3) ArUco marker ──
                if self.aruco_available:
                    try:
                        corners, ids, _ = self.aruco_detector.detectMarkers(frame)
                        if ids is not None and len(ids) > 0:
                            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                            self.marker_data = {
                                'ids': ids.flatten().tolist(),
                                'count': len(ids)
                            }
                        else:
                            self.marker_data = {'ids': [], 'count': 0}
                    except Exception:
                        pass

                # ── Overlay çizim ──
                frame = self.line_detector.draw_overlay(frame, self.line_result)
                frame = self.qr_reader.draw_overlay(frame, self.qr_result)

            # OSD
            cv2.putText(frame, "AVTECH AGV", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 200), 2)
            cv2.putText(frame, time.strftime("%H:%M:%S"), (self.width - 100, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            with self.frame_lock:
                self.frame = frame

            time.sleep(0.001)  # CPU rahatlatma

    def get_frame(self):
        """Son yakalanan frame'i döndür."""
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None

    def get_jpeg(self, quality=70):
        """Son frame'i JPEG olarak döndür."""
        with self.frame_lock:
            if self.frame is None:
                return None
            _, jpeg = cv2.imencode('.jpg', self.frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, quality])
            return jpeg.tobytes()

    def generate_mjpeg(self):
        """Flask için MJPEG stream generator."""
        while True:
            jpeg = self.get_jpeg(quality=65)
            if jpeg:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
            time.sleep(0.033)  # ~30 FPS

    def get_markers(self):
        """ArUco marker bilgileri."""
        return self.marker_data

    def get_line_result(self):
        """Çizgi algılama sonucu."""
        return self.line_result

    def get_qr_result(self):
        """QR kod okuma sonucu."""
        return self.qr_result


# --- Test ---

if __name__ == '__main__':
    cam = CameraNode(camera_index=0, width=640, height=480)

    if cam.start():
        print("Kamera çalışıyor. 'q' ile çıkış...")
        try:
            while True:
                frame = cam.get_frame()
                if frame is not None:
                    cv2.imshow("AVTECH AGV Camera", frame)

                    lr = cam.get_line_result()
                    if lr.get('detected'):
                        print(f"  LINE: {lr['color']} offset={lr['offset_cm']:.1f}cm "
                              f"conf={lr['confidence']:.0%}")

                    qr = cam.get_qr_result()
                    if qr.get('detected'):
                        print(f"  QR: '{qr['data']}' dist={qr['distance_m']:.2f}m")

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                time.sleep(0.033)
        except KeyboardInterrupt:
            pass
        finally:
            cam.stop()
            cv2.destroyAllWindows()
    else:
        print("Kamera başlatılamadı!")
