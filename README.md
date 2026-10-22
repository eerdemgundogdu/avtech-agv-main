# AVTECH AGV: Forklift Robot

Software for a line-following forklift AGV built by ÇÜ AVTECH for the *Sanayide Robotik Uygulamalar 2026* competition. The vehicle uses a hoverboard BLDC drivetrain, a Teensy 4.1 for real-time control and a Jetson Orin Nano as its high-level computer.

The current build (motion-capability video stage) is driven manually from a web GUI. The autonomy modules (camera line detection, QR task reading, sensor fusion, mapping, mission control) are in the repo and can be run standalone, but they are not wired into the running system yet.

```
 Browser GUI  ──Socket.IO──►  Jetson Orin Nano  ──USB serial, JSON @115200──►  Teensy 4.1
 (joystick/WASD)  ◄──telemetry──  Flask server   ◄──────telemetry 10 Hz───────  real-time control
                                                                                 │  │  │
                                                    Hoverboard FOC (UART 0xABCD)─┘  │  └─ DM556 stepper (forklift)
                                                    ICM-20948 IMU (SPI, 100 Hz)─────┘
```

## Teensy firmware (`teensy_firmware/`)

- **Non-blocking, time-sliced main loop:** IMU at 100 Hz, motor commands at 20 Hz, telemetry at 10 Hz. No `delay()` calls on the control path.
- **Hoverboard driver:** speaks the EFeru FOC firmware's USART protocol (speed + steer frames, `0xABCD` start, XOR checksum) and parses feedback frames.
- **IMU:** ICM-20948 over SPI with gyro bias calibration and a complementary filter for attitude/yaw.
- **Heading hold:** while driving straight, a PI controller with gyro-rate damping locks the yaw at 50 Hz. It re-targets when the error diverges.
- **Forklift:** DM556 stepper pulsed from an `IntervalTimer` ISR with position tracking, soft limits, homing and a hold-to-move keepalive.
- **Navigation FSM:** IDLE, MANUAL, LINE_FOLLOW, TURN, PICKUP/DROP, OBSTACLE_STOP and FAULT, plus a line-following PID (QTR-8A or a camera offset sent by the Jetson).
- **Safety:** a manual-command timeout with ramp-down, a Jetson link watchdog driven by heartbeats, motor outputs zeroed when inactive, and an E-stop path.
- **Command parser:** a hand-written, allocation-free JSON line parser with bounded buffers.
- `comm_test/` holds single-subsystem bring-up sketches (IMU, QTR, stepper, ultrasonic, UART, safety) and a PC-side test tool.

All pins, gains and timings are in `config.h`.

## Jetson software (`jetson_software/`)

- `avtech_brain.py`: a Flask + Socket.IO server. It resends GUI drive input to the Teensy at a fixed 20 Hz so Wi-Fi jitter doesn't reach the motors, applies a deadman timeout and offers three speed modes.
- `teensy_link.py`: serial link with PJRC VID auto-detection, an exclusive port lock, auto-reconnect and a 5 Hz heartbeat.
- Autonomy modules, each runnable on its own:
  - `camera_node.py`: HSV color-line detection with a perspective (bird's-eye) transform and ArUco markers.
  - `qr_reader.py`: reads pick/drop task QR codes.
  - `ekf_localization.py`: Extended Kalman Filter over `[x, y, θ, gyro_bias]`. Wheel odometry and gyro drive the prediction step. The wheel-derived yaw rate makes the gyro bias observable, and known QR landmarks give range/bearing updates, with a Mahalanobis gate that rejects outliers.
  - `sensor_fusion.py`: wraps the EKF (default) or the older complementary filter (`use_ekf=False`) and adds line/obstacle state.
  - `mapping_node.py`: 2D occupancy grid built from odometry and ultrasonic sensors.
  - `mission_control.py`: task sequencing.
- `ekf_test.py`: square-course simulation with a biased gyro. Final position error is ~0.06 m with the EKF versus ~0.54 m with dead reckoning, and the gyro bias estimate converges.
- Tools: `calibrate_camera.py` (checkerboard calibration) and `jetson_hsv_tuner.py` (live threshold tuning).

## Running

Teensy: open `teensy_firmware/teensy_firmware.ino` in the Arduino IDE with Teensyduino and flash it to a Teensy 4.1.

Jetson:

```bash
cd jetson_software
./install_jetson.sh        # installs dependencies and the systemd service
python3 avtech_brain.py    # GUI at http://<jetson-ip>:5000
```

| Key | Action |
|-----|--------|
| W A S D / arrows | drive |
| 1 / 2 / 3 | slow / medium / fast |
| Space / Esc | stop |
| E / Q | forklift up / down (hold) |
| T / G | forklift to top / bottom |
| H | toggle heading hold |

Code comments and log strings are in Turkish.
