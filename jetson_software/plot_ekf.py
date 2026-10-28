#!/usr/bin/env python3
# README icin EKF lokalizasyon grafigini uretir (ekf_test.py simulasyonu).
#
#   pip install numpy matplotlib
#   python3 plot_ekf.py        # -> ../docs/img/ekf_localization.png

import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from ekf_test import DT, LANDMARKS, run  # noqa: E402

BLUE, ORANGE = '#2a78d6', '#eb6834'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs',
                   'img', 'ekf_localization.png')


def main():
    trace = []
    err_ekf, err_dr, _ = run(trace=trace)
    tr = np.array(trace)
    t = np.arange(len(err_ekf)) * DT

    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID,
                         'axes.labelcolor': INK2, 'xtick.color': INK2,
                         'ytick.color': INK2})
    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(10.5, 4.6), gridspec_kw={'width_ratios': [1, 1.25],
                                                'wspace': 0.28})
    fig.patch.set_facecolor('white')

    ax1.plot(tr[:, 2], tr[:, 3], color=ORANGE, lw=1.5, label='Dead reckoning')
    ax1.plot(tr[:, 4], tr[:, 5], color=BLUE, lw=2.0, label='EKF')
    ax1.plot(tr[:, 0], tr[:, 1], color=INK, lw=1.2, ls='--', label='Ground truth')
    lx, ly = zip(*LANDMARKS.values())
    ax1.scatter(lx, ly, marker='s', s=60, facecolor='white', edgecolor=INK,
                linewidth=1.2, zorder=5, label='QR landmark')
    ax1.set_aspect('equal')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_xlim(-0.6, 2.8)
    ax1.set_ylim(-0.6, 2.8)
    ax1.set_title('2 m square course, 0.8°/s gyro bias', color=INK,
                  fontsize=10, loc='left')
    ax1.legend(frameon=False, loc='upper center', fontsize=8.5, ncol=2,
               bbox_to_anchor=(0.5, -0.14))

    ax2.plot(t, err_dr, color=ORANGE, lw=1.5,
             label=f'Dead reckoning  (final {err_dr[-1]:.2f} m)')
    ax2.plot(t, err_ekf, color=BLUE, lw=2.0,
             label=f'EKF  (final {err_ekf[-1]:.2f} m)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position error (m)')
    ax2.set_ylim(0, None)
    ax2.set_title('Position error over the run', color=INK, fontsize=10,
                  loc='left')
    ax2.legend(frameon=False, loc='upper left', fontsize=9)

    for ax in (ax1, ax2):
        ax.grid(True, color=GRID, lw=0.6)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)

    ratio = err_dr[-1] / err_ekf[-1]
    fig.suptitle(f'EKF localization: final position error {err_dr[-1]:.2f} m → '
                 f'{err_ekf[-1]:.2f} m ({ratio:.0f}× lower) in simulation',
                 color=INK, fontsize=11.5, x=0.06, ha='left', y=1.0)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150, bbox_inches='tight')
    print(os.path.normpath(OUT), math.isfinite(ratio))


if __name__ == '__main__':
    main()
