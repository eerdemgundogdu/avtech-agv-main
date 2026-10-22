#!/usr/bin/env python3
# Donanimsiz EKF testi: biasli gyro + gurultulu odometri ile kare parkur.
# Sadece odometri/gyro entegrasyonu (dead reckoning) ile EKF karsilastirilir.
#
#   python3 ekf_test.py

import math

import numpy as np

from ekf_localization import EkfLocalization, wrap_angle

WHEEL_BASE = 0.45
DT = 0.05                     # 20 Hz telemetri
GYRO_BIAS = math.radians(0.8) # 0.8 deg/s bias
LANDMARKS = {'q1': (2.0, 0.0), 'q2': (2.0, 2.0), 'q3': (0.0, 2.0),
             'q4': (0.0, 0.0)}


def square_path():
    """Kare parkur: 2 m ileri, yerinde 90 derece don, x4."""
    cmds = []
    for _ in range(4):
        cmds += [(0.4, 0.0)] * int(2.0 / 0.4 / DT)
        cmds += [(0.0, math.radians(45))] * int(2.0 / DT)
    return cmds


def run(seed=0):
    rng = np.random.default_rng(seed)
    ekf = EkfLocalization()
    true = np.zeros(3)
    dr = np.zeros(3)
    err_ekf, err_dr = [], []

    for k, (v, w) in enumerate(square_path()):
        # gercek hareket
        true[0] += v * math.cos(true[2]) * DT
        true[1] += v * math.sin(true[2]) * DT
        true[2] = wrap_angle(true[2] + w * DT)

        # sensorler
        vl = v - w * WHEEL_BASE / 2 + rng.normal(0, 0.03)
        vr = v + w * WHEEL_BASE / 2 + rng.normal(0, 0.03)
        v_odom = (vl + vr) / 2
        odom_rate = (vr - vl) / WHEEL_BASE
        gyro = w + GYRO_BIAS + rng.normal(0, 0.01)

        # dead reckoning (gyro entegrasyonu + odometri hizi)
        dr[0] += v_odom * math.cos(dr[2]) * DT
        dr[1] += v_odom * math.sin(dr[2]) * DT
        dr[2] = wrap_angle(dr[2] + gyro * DT)

        # EKF
        ekf.predict(v_odom, gyro, DT)
        ekf.update_odom_rate(odom_rate, gyro)

        # her 2 s'de bir 1.5 m icindeki QR kodu gor
        if k % 40 == 0:
            for lx, ly in LANDMARKS.values():
                dx, dy = lx - true[0], ly - true[1]
                rng_m = math.hypot(dx, dy)
                if 0.2 < rng_m < 1.5:
                    bearing = wrap_angle(math.atan2(dy, dx) - true[2])
                    ekf.update_landmark(lx, ly,
                                        rng_m + rng.normal(0, 0.03),
                                        bearing + rng.normal(0, math.radians(2)))

        px, py, _ = ekf.pose
        err_ekf.append(math.hypot(px - true[0], py - true[1]))
        err_dr.append(math.hypot(dr[0] - true[0], dr[1] - true[1]))

    return np.array(err_ekf), np.array(err_dr), ekf


def test_ekf():
    err_ekf, err_dr, ekf = run()
    print(f'Son konum hatasi   dead reckoning: {err_dr[-1]:.3f} m   '
          f'EKF: {err_ekf[-1]:.3f} m')
    print(f'Ortalama hata      dead reckoning: {err_dr.mean():.3f} m   '
          f'EKF: {err_ekf.mean():.3f} m')
    print(f'Gyro bias kestirimi: {math.degrees(ekf.gyro_bias):.2f} deg/s '
          f'(gercek {math.degrees(GYRO_BIAS):.2f})')

    assert err_ekf[-1] < 0.5 * err_dr[-1]
    assert abs(ekf.gyro_bias - GYRO_BIAS) < math.radians(0.2)

    # Aykiri olcum kapida reddedilmeli
    before = ekf.pose
    accepted = ekf.update_landmark(10.0, 10.0, 0.3, 0.0)
    assert not accepted and ekf.pose == before


if __name__ == '__main__':
    test_ekf()
    print('OK')
