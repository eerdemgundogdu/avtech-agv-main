#!/usr/bin/env python3
# ============================================================================
#  AVTECH AGV — Sensor Fusion Modülü
#  IMU + Odometri + Kamera + QR Landmark kaynaştırma
#
#  Varsayilan: EKF (ekf_localization.py) - odometri + gyro tahmini,
#  odometri acisal hizi ile gyro bias kestirimi, QR landmark ile
#  mesafe/aci guncellemesi.
#  use_ekf=False: eski complementary filter yolu.
# ============================================================================

import math
import time
import numpy as np

from ekf_localization import EkfLocalization


class RobotState:
    """Birleştirilmiş robot durumu."""

    __slots__ = [
        'x', 'y', 'heading',       # Pozisyon (m) ve yön (derece)
        'vx', 'vy', 'speed',       # Hız bileşenleri (m/s)
        'omega',                     # Açısal hız (°/s)
        'line_offset_ir',           # IR çizgi sapma (sensör birimi)
        'line_offset_cam',          # Kamera çizgi sapma (cm)
        'line_detected_ir',         # IR çizgi algılandı mı
        'line_detected_cam',        # Kamera çizgi algılandı mı
        'qr_id', 'qr_distance',    # QR kod bilgisi
        'qr_angle', 'qr_offset',   # QR açı ve sapma
        'obs_front', 'obs_left', 'obs_right',  # Engel mesafeleri (cm)
        'battery_v',                # Batarya voltajı
        'timestamp',                # Son güncelleme zamanı
    ]

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0
        self.omega = 0.0
        self.line_offset_ir = 0.0
        self.line_offset_cam = 0.0
        self.line_detected_ir = False
        self.line_detected_cam = False
        self.qr_id = ''
        self.qr_distance = 0.0
        self.qr_angle = 0.0
        self.qr_offset = 0.0
        self.obs_front = 200.0
        self.obs_left = 200.0
        self.obs_right = 200.0
        self.battery_v = 0.0
        self.timestamp = 0.0

    def to_dict(self):
        return {k: getattr(self, k) for k in self.__slots__}


class SensorFusion:
    """Complementary filter tabanlı sensor fusion.

    Veri kaynakları:
    1. IMU yaw (BNO055/ICM-20948) — hızlı ama drift'e açık (gyro entegrasyonu)
    2. Odometri (hoverboard RPM) — drift var ama uzun vadeli stabil
    3. QR kod landmark — mutlak pozisyon düzeltmesi
    4. Kamera çizgi — yol sapması bilgisi
    """

    def __init__(self, wheel_base_m=0.45, wheel_diameter_m=0.165,
                 use_ekf=True):
        """
        Args:
            wheel_base_m: Tekerlekler arası mesafe (metre)
            wheel_diameter_m: Tekerlek çapı (metre)
        """
        self.wheel_base = wheel_base_m
        self.wheel_circumference = wheel_diameter_m * math.pi
        self.rpm_to_mps = self.wheel_circumference / 60.0

        # Robot durumu
        self.state = RobotState()

        self.use_ekf = use_ekf
        self.ekf = EkfLocalization() if use_ekf else None

        # Complementary filter katsayıları
        # Yüksek alpha → IMU'ya güven (kısa vadeli doğru)
        # Düşük alpha → odometriye güven (uzun vadeli stabil)
        self.alpha_heading = 0.95  # Heading: %95 IMU, %5 odometri
        self.alpha_position = 0.7  # Pozisyon: %70 odometri, %30 IMU

        # QR landmark veritabanı — {qr_id: (x, y)} harita koordinatları
        self.qr_landmarks = {}

        # Son veriler
        self._last_imu_yaw = 0.0
        self._last_odom_heading = 0.0
        self._last_speed_l = 0.0
        self._last_speed_r = 0.0
        self._last_update = time.time()

        # Trajectory (iz)
        self.trajectory = []
        self._traj_min_dist = 0.05  # Her 5cm'de bir kaydet

        # Drift düzeltme offset
        self._heading_offset = 0.0
        self._position_offset_x = 0.0
        self._position_offset_y = 0.0

        print("[FUSION] Sensor fusion initialized")
        print(f"[FUSION]   Wheel base: {wheel_base_m:.3f}m")
        print(f"[FUSION]   Wheel circumference: {self.wheel_circumference:.4f}m")
        print(f"[FUSION]   Mode: {'EKF' if use_ekf else 'complementary'}")

    def register_qr_landmark(self, qr_id, x, y):
        """QR kod landmark pozisyonunu kaydet.

        Args:
            qr_id: QR kod string içeriği
            x, y: Harita koordinatı (metre)
        """
        self.qr_landmarks[qr_id] = (x, y)
        print(f"[FUSION] QR landmark registered: '{qr_id}' at ({x:.2f}, {y:.2f})")

    def update(self, telemetry, qr_result=None, camera_line=None):
        """Tüm sensör verilerini birleştir ve durumu güncelle.

        Args:
            telemetry: Teensy'den gelen telemetri dict
                {yaw, gyroZ, spdL, spdR, bat, obsFront, obsLeft, obsRight,
                 line, linePos, ...}
            qr_result: QRReader.process() çıktısı (veya None)
            camera_line: Kamera çizgi algılama sonucu
                {'detected': bool, 'offset_cm': float, 'color': str}
        """
        now = time.time()
        dt = now - self._last_update
        if dt > 1.0:
            dt = 0.05  # İlk çağrı veya uzun boşluk
        self._last_update = now
        self.state.timestamp = now

        # ── 1) IMU Heading ──
        imu_yaw = telemetry.get('yaw', self._last_imu_yaw)
        gyro_z = telemetry.get('gyroZ', 0.0)
        self._last_imu_yaw = imu_yaw
        self.state.omega = gyro_z

        # ── 2) Odometri (diferansiyel sürüş) ──
        speed_l_rpm = telemetry.get('spdL', 0)
        speed_r_rpm = telemetry.get('spdR', 0)

        v_left = speed_l_rpm * self.rpm_to_mps
        v_right = speed_r_rpm * self.rpm_to_mps

        # Doğrusal ve açısal hız
        v_linear = (v_left + v_right) / 2.0
        v_angular_rad = (v_right - v_left) / self.wheel_base  # rad/s

        # Odometriden heading tahmini
        odom_delta_heading = math.degrees(v_angular_rad * dt)
        self._last_odom_heading += odom_delta_heading

        # Heading normalizasyon
        while self._last_odom_heading >= 360.0:
            self._last_odom_heading -= 360.0
        while self._last_odom_heading < 0.0:
            self._last_odom_heading += 360.0

        if self.use_ekf:
            self._update_ekf(dt, v_linear, v_angular_rad, gyro_z, qr_result)
        else:
            self._update_complementary(dt, imu_yaw, v_linear, qr_result)

        self._update_aux(telemetry, qr_result, camera_line)
        return self.state

    def _update_ekf(self, dt, v_linear, odom_rate, gyro_z_dps, qr_result):
        gyro = math.radians(gyro_z_dps)
        self.ekf.predict(v_linear, gyro, dt)
        self.ekf.update_odom_rate(odom_rate, gyro)

        if qr_result and qr_result.get('detected', False):
            qr_id = qr_result['data']
            dist = qr_result.get('distance_m', 0)
            if qr_id in self.qr_landmarks and dist > 0.05:
                lm_x, lm_y = self.qr_landmarks[qr_id]
                ok = self.ekf.update_landmark(
                    lm_x, lm_y, dist,
                    math.radians(qr_result.get('angle_deg', 0)))
                if not ok:
                    print(f"[FUSION] QR '{qr_id}' rejected by EKF gate")

        px, py, th = self.ekf.pose
        self.state.x = px
        self.state.y = py
        self.state.heading = math.degrees(th) % 360.0
        self.state.vx = v_linear * math.cos(th)
        self.state.vy = v_linear * math.sin(th)
        self.state.speed = abs(v_linear)

    def _update_complementary(self, dt, imu_yaw, v_linear, qr_result):
        # ── 3) Complementary Filter — Heading ──
        # IMU yaw doğrudan derece veriyor (BNO055 sensor fusion)
        # Odometri heading kümülatif hesaplama
        # Complementary: alpha * IMU + (1-alpha) * odometri

        # Açı farkını hesapla (en kısa yolu bul)
        imu_heading = imu_yaw + self._heading_offset
        while imu_heading >= 360.0:
            imu_heading -= 360.0
        while imu_heading < 0.0:
            imu_heading += 360.0

        # Weighted average (circular mean)
        diff = imu_heading - self._last_odom_heading
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0

        fused_heading = self._last_odom_heading + self.alpha_heading * diff
        while fused_heading >= 360.0:
            fused_heading -= 360.0
        while fused_heading < 0.0:
            fused_heading += 360.0

        self.state.heading = fused_heading

        # ── 4) Pozisyon Güncelleme ──
        heading_rad = math.radians(self.state.heading)
        dx = v_linear * math.cos(heading_rad) * dt
        dy = v_linear * math.sin(heading_rad) * dt

        self.state.x += dx + self._position_offset_x
        self.state.y += dy + self._position_offset_y
        self._position_offset_x = 0.0
        self._position_offset_y = 0.0

        self.state.vx = v_linear * math.cos(heading_rad)
        self.state.vy = v_linear * math.sin(heading_rad)
        self.state.speed = abs(v_linear)

        # ── 5) QR Landmark Düzeltmesi ──
        if qr_result and qr_result.get('detected', False):
            qr_id = qr_result['data']

            # Bilinen landmark'sa pozisyon düzelt
            if qr_id in self.qr_landmarks:
                lm_x, lm_y = self.qr_landmarks[qr_id]
                # QR kodun kameraya göre pozisyonundan robotun gerçek pozisyonunu hesapla
                qr_dist = qr_result.get('distance_m', 0)
                qr_angle_deg = qr_result.get('angle_deg', 0)

                if qr_dist > 0.05:  # En az 5cm mesafede olmalı
                    # Robot pozisyonunu QR koddan geriye hesapla
                    qr_angle_rad = math.radians(self.state.heading + qr_angle_deg)
                    estimated_x = lm_x - qr_dist * math.cos(qr_angle_rad)
                    estimated_y = lm_y - qr_dist * math.sin(qr_angle_rad)

                    # Yumuşak düzeltme (anlık atlama yok)
                    correction_gain = 0.3  # %30 düzeltme uygula
                    self._position_offset_x = (estimated_x - self.state.x) * correction_gain
                    self._position_offset_y = (estimated_y - self.state.y) * correction_gain

                    print(f"[FUSION] QR landmark correction: '{qr_id}' → "
                          f"Δx={self._position_offset_x:.3f}m, Δy={self._position_offset_y:.3f}m")

    def _update_aux(self, telemetry, qr_result, camera_line):
        if qr_result and qr_result.get('detected', False):
            self.state.qr_id = qr_result['data']
            self.state.qr_distance = qr_result.get('distance_m', 0)
            self.state.qr_angle = qr_result.get('angle_deg', 0)
            self.state.qr_offset = qr_result.get('offset_x_cm', 0)
        else:
            self.state.qr_id = ''

        # ── 6) Çizgi Sensör Verileri ──
        # IR çizgi (Teensy'den)
        line_state = telemetry.get('line', 'LOST_ALL')
        self.state.line_detected_ir = (line_state == 'ON_LINE')
        self.state.line_offset_ir = telemetry.get('linePos', 0.0)

        # Kamera çizgi
        if camera_line and camera_line.get('detected', False):
            self.state.line_detected_cam = True
            self.state.line_offset_cam = camera_line.get('offset_cm', 0.0)
        else:
            self.state.line_detected_cam = False

        # ── 7) Engel Verileri ──
        self.state.obs_front = telemetry.get('obsFront', 200)
        self.state.obs_left = telemetry.get('obsLeft', 200)
        self.state.obs_right = telemetry.get('obsRight', 200)

        # ── 8) Batarya ──
        self.state.battery_v = telemetry.get('bat', 0)

        # ── 9) Trajectory Kaydet ──
        if len(self.trajectory) == 0:
            self.trajectory.append((self.state.x, self.state.y))
        else:
            last_x, last_y = self.trajectory[-1]
            dist = math.sqrt((self.state.x - last_x)**2 + (self.state.y - last_y)**2)
            if dist >= self._traj_min_dist:
                self.trajectory.append((self.state.x, self.state.y))

        # Trajectory uzunluk limiti
        if len(self.trajectory) > 2000:
            self.trajectory = self.trajectory[-1000:]

    def get_fused_line_offset(self):
        """IR + kamera çizgi sapmasını birleştir.

        IR sensör daha hızlı (doğrudan Teensy'de), kamera daha doğru.
        Complementary filter ile birleştir.

        Returns:
            float: Birleşik çizgi sapması (cm cinsinden)
            str: Aktif kaynak ('ir', 'camera', 'both', 'none')
        """
        ir_ok = self.state.line_detected_ir
        cam_ok = self.state.line_detected_cam

        if ir_ok and cam_ok:
            # Her ikisi de aktif — kameraya daha fazla güven
            alpha_cam = 0.7
            # IR offset'i cm'ye çevir (yaklaşık: sensör ağırlık birimi * 2cm)
            ir_cm = self.state.line_offset_ir * 2.0
            fused = alpha_cam * self.state.line_offset_cam + (1 - alpha_cam) * ir_cm
            return fused, 'both'
        elif cam_ok:
            return self.state.line_offset_cam, 'camera'
        elif ir_ok:
            return self.state.line_offset_ir * 2.0, 'ir'
        else:
            return 0.0, 'none'

    def get_state(self):
        """Güncel robot durumunu döndür."""
        return self.state

    def get_state_dict(self):
        """Robot durumunu dict olarak döndür (JSON serializable)."""
        d = self.state.to_dict()
        # Float'ları yuvarla
        for k, v in d.items():
            if isinstance(v, float):
                d[k] = round(v, 4)
        d['trajectory_len'] = len(self.trajectory)
        d['trajectory'] = [(round(x, 3), round(y, 3))
                          for x, y in self.trajectory[-100:]]
        return d

    def reset(self):
        """Durumu sıfırla."""
        self.state = RobotState()
        self.trajectory.clear()
        self._last_odom_heading = 0.0
        self._heading_offset = 0.0
        self._position_offset_x = 0.0
        self._position_offset_y = 0.0
        if self.use_ekf:
            self.ekf = EkfLocalization()
        print("[FUSION] State reset")


# --- Test ---

if __name__ == '__main__':
    fusion = SensorFusion(wheel_base_m=0.45, wheel_diameter_m=0.165)

    # QR landmark'ları kaydet
    fusion.register_qr_landmark("q1", 2.0, 0.0)
    fusion.register_qr_landmark("q2", 4.0, 0.0)
    fusion.register_qr_landmark("q3", 4.0, 3.0)

    # Simüle telemetri
    for i in range(200):
        telemetry = {
            'yaw': (i * 0.5) % 360,
            'gyroZ': 0.5,
            'spdL': 60,
            'spdR': 62,
            'bat': 24.2,
            'obsFront': 120,
            'obsLeft': 80,
            'obsRight': 90,
            'line': 'ON_LINE',
            'linePos': math.sin(i * 0.1) * 1.5,
        }

        # QR simüle (her 50. adımda)
        qr = None
        if i == 50:
            qr = {'detected': True, 'data': 'q1', 'distance_m': 0.5,
                   'angle_deg': 5.0, 'offset_x_cm': 2.0}

        state = fusion.update(telemetry, qr_result=qr)
        time.sleep(0.02)

    s = fusion.get_state()
    print(f"\nFinal State:")
    print(f"  Position: ({s.x:.3f}, {s.y:.3f})m")
    print(f"  Heading:  {s.heading:.1f}°")
    print(f"  Speed:    {s.speed:.3f} m/s")
    print(f"  Battery:  {s.battery_v:.1f}V")
    print(f"  Trajectory points: {len(fusion.trajectory)}")
