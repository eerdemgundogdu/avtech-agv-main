# AVTECH AGV: Forklift Robot

Software and carrier PCB design for a line-following forklift AGV built by CU AVTECH for the Sanayide Robotik Uygulamalar 2026 competition. The vehicle uses a hoverboard BLDC drivetrain, a Teensy 4.1 for real-time control and a Jetson Orin Nano as its high-level computer.

## Teensy firmware (`teensy_firmware/`)

- Non-blocking, time-sliced main loop: IMU at 100 Hz, motor commands at 20 Hz, telemetry at 10 Hz. No `delay()` calls on the control path.
- Hoverboard driver: speaks the EFeru FOC firmware's USART protocol (speed + steer frames, `0xABCD` start, XOR checksum) and parses feedback frames. Framing, checksum and speed ramping live in `hover_protocol.cpp`, unit-tested on the host.
- IMU: ICM-20948 over SPI with gyro bias calibration and a complementary filter for attitude/yaw.
- Heading hold: PI controller with gyro-rate damping locks the yaw at 50 Hz while driving straight.
- Forklift: DM556 stepper pulsed from an `IntervalTimer` ISR with position tracking, soft limits, homing and a hold-to-move keepalive.
- Navigation FSM: IDLE, MANUAL, LINE_FOLLOW, TURN, PICKUP/DROP, OBSTACLE_STOP and FAULT, plus a line-following PID (QTR-8A or camera offset from Jetson).
- Safety: manual-command timeout with ramp-down, Jetson link watchdog driven by heartbeats, motor outputs zeroed when inactive, and hardware/software E-stop path.
- Command parser: allocation-free JSON line parser with bounded buffers (`cmd_parser.cpp`, host-tested).

All pins, gains and timings are in `config.h`.

## Jetson software (`jetson_software/`)

- `avtech_brain.py`: Flask + Socket.IO control server sending drive input to the Teensy at a fixed 20 Hz with deadman timeout.
- `teensy_link.py`: serial link with PJRC VID auto-detection, port lock, auto-reconnect and 5 Hz heartbeat.
- `camera_node.py`: HSV color-line detection with perspective (bird's-eye) transform and ArUco markers.
- `qr_reader.py`: pick/drop task QR code reader.
- `ekf_localization.py`: Extended Kalman Filter over `[x, y, theta, gyro_bias]` fusing wheel odometry, gyro and QR landmark range/bearing measurements with Mahalanobis outlier gating (`ekf_test.py`).
- `sensor_fusion.py`: wraps the EKF and adds line/obstacle state.
- `mapping_node.py`: 2D occupancy grid built from odometry and ultrasonic sensors.
- `mission_control.py`: task sequencing.

## Carrier board (`hardware/`)

A 2-layer KiCad carrier board for the Teensy 4.1 (Rev A, 105x80 mm). It replaces hand wiring with a 24 V input stage (fuse, reverse-polarity diode, SMBJ30CA TVS), LMR16006X 24 V to 5 V buck, AP2112K-3.3 sensor LDO, BSS138 level shifters, HC-SR04 echo dividers, DM556 open-drain drivers, and a hardware E-stop contactor loop.

The schematic and PCB are generated from `hardware/gen/design.py` and checked with KiCad DRC/ERC. Fabrication outputs (Gerber, drill, BOM, pick-and-place, schematic PDF) are in `hardware/output/`. Design calculations: `hardware/DESIGN.md`; connector pinouts: `hardware/WIRING.md`.

## Testing and Build

```bash
cmake -S firmware_tests -B build/test && cmake --build build/test && ctest --test-dir build/test
cd jetson_software && python3 ekf_test.py
pio run -e teensy41_carrier
```
