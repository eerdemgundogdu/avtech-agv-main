#!/usr/bin/env python3
# ============================================================================
#  AVTECH AGV - EKF tabanli konum kestirimi
#
#  Durum: x = [px, py, theta, b]
#    px, py : harita konumu (m)
#    theta  : yon (rad, saat yonu tersi pozitif)
#    b      : gyro Z sapmasi / bias (rad/s)
#
#  Tahmin (her telemetri paketinde):
#    giris: v  = tekerlek odometrisinden dogrusal hiz (m/s)
#           wg = IMU gyro Z (rad/s)
#    px' = px + v cos(theta) dt
#    py' = py + v sin(theta) dt
#    th' = th + (wg - b) dt
#    b'  = b
#
#  Guncellemeler:
#    1) Odometri acisal hizi: z = (vR - vL) / L,  h(x) = wg - b
#       -> gyro bias'ini gozlemlenebilir yapar (duz giderken tekerlek
#          farki ~0, gyro ise bias kadar donuyor gorunur).
#    2) QR landmark (bilinen konum): z = [mesafe, aci]
#       h(x) = [ sqrt(dx^2 + dy^2),  atan2(dy, dx) - theta ]
#       -> pozisyon ve yon drift'ini mutlak olarak duzeltir.
#
#  Gurultu varsayilanlari masa ve saha testlerinden kaba ayarlandi;
#  tekerlek kaymasi olan zeminde odometri R'si artirilmali.
# ============================================================================

import math

import numpy as np


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class EkfLocalization:

    def __init__(self,
                 sigma_v=0.05,          # odometri hiz gurultusu (m/s)
                 sigma_gyro=0.02,       # gyro gurultusu (rad/s)
                 sigma_bias=0.0005,     # bias random walk (rad/s/sqrt(s))
                 sigma_odom_rate=0.08,  # odometri acisal hiz olcumu (rad/s)
                 sigma_range=0.05,      # QR mesafe olcumu (m)
                 sigma_bearing=math.radians(3.0)):
        self.x = np.zeros(4)
        self.P = np.diag([0.01, 0.01, math.radians(2.0) ** 2, 0.01 ** 2])

        self.sigma_v = sigma_v
        self.sigma_gyro = sigma_gyro
        self.sigma_bias = sigma_bias
        self.R_rate = np.array([[sigma_odom_rate ** 2]])
        self.R_qr = np.diag([sigma_range ** 2, sigma_bearing ** 2])

        # Mahalanobis kapisi: aykiri QR okumalarini (yanlis eslesme,
        # bulanik kare) reddet. 2 serbestlik derecesi, %99.
        self.qr_gate = 9.21
        self.rejected_qr = 0

    # --- durum erisimi ---

    def set_pose(self, px, py, theta):
        self.x[0:3] = [px, py, wrap_angle(theta)]

    @property
    def pose(self):
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    @property
    def gyro_bias(self):
        return float(self.x[3])

    # --- tahmin ---

    def predict(self, v, gyro_z, dt):
        if dt <= 0.0:
            return
        px, py, th, b = self.x
        w = gyro_z - b
        c, s = math.cos(th), math.sin(th)

        self.x = np.array([px + v * c * dt,
                           py + v * s * dt,
                           wrap_angle(th + w * dt),
                           b])

        F = np.array([[1.0, 0.0, -v * s * dt, 0.0],
                      [0.0, 1.0,  v * c * dt, 0.0],
                      [0.0, 0.0,  1.0,       -dt],
                      [0.0, 0.0,  0.0,        1.0]])

        # Giris gurultusu (v, wg) durum uzayina G ile tasinir
        G = np.array([[c * dt, 0.0],
                      [s * dt, 0.0],
                      [0.0,    dt],
                      [0.0,    0.0]])
        Qu = np.diag([self.sigma_v ** 2, self.sigma_gyro ** 2])
        Q = G @ Qu @ G.T
        Q[3, 3] += self.sigma_bias ** 2 * dt

        self.P = F @ self.P @ F.T + Q

    # --- guncellemeler ---

    def _update(self, y, H, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return S

    def update_odom_rate(self, odom_rate, gyro_z):
        """Tekerlek farkindan hesaplanan acisal hiz ile bias'i gozle."""
        H = np.array([[0.0, 0.0, 0.0, -1.0]])
        y = np.array([odom_rate - (gyro_z - self.x[3])])
        self._update(y, H, self.R_rate)

    def update_landmark(self, lm_x, lm_y, meas_range, meas_bearing):
        """Bilinen konumdaki QR koddan mesafe/aci olcumu.

        meas_bearing: robotun burnuna gore aci (rad, sola pozitif).
        Kapidan gecemeyen olcum reddedilir; True/False doner.
        """
        dx = lm_x - self.x[0]
        dy = lm_y - self.x[1]
        q = dx * dx + dy * dy
        if q < 1e-6:
            return False
        r = math.sqrt(q)

        h = np.array([r, wrap_angle(math.atan2(dy, dx) - self.x[2])])
        H = np.array([[-dx / r, -dy / r,  0.0, 0.0],
                      [ dy / q, -dx / q, -1.0, 0.0]])
        y = np.array([meas_range - h[0], wrap_angle(meas_bearing - h[1])])

        S = H @ self.P @ H.T + self.R_qr
        d2 = float(y @ np.linalg.solve(S, y))
        if d2 > self.qr_gate:
            self.rejected_qr += 1
            return False

        self._update(y, H, self.R_qr)
        return True
