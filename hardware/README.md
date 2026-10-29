# AVTECH AGV Carrier Board (Rev A)

2-layer (105x80 mm, FR-4, 1.6 mm, 1 oz Cu) carrier board for the Teensy 4.1 forklift AGV.

Subsystems on the board:

- Power input: 1 A fuse, SS16 reverse-polarity diode, SMBJ30CA TVS, 47 uF bulk capacitor.
- 24 V to 5 V buck: LMR16006X (60 V, 0.6 A, 700 kHz), 22 uH inductor.
- 5 V to 3.3 V LDO: AP2112K-3.3 for sensors; Teensy VIN isolated through a B5819W Schottky diode.
- Hardware E-stop: NC button in series with the motor-power contactor coil, plus `MOTOR_EN` low-side switch (`BCP56-16`) and loop sense divider.
- Hoverboard UART: 3-pin connector (15 V pin omitted), 1k series resistors and BAT54S Schottky clamps.
- HC-SR04 x3: 5 V to 3.0 V echo voltage dividers.
- QTR-8A: powered from 3.3 V to keep analog outputs within the Teensy 4.1 ADC range.
- DM556 stepper: common-anode 5 V drive through AO3400A low-side MOSFETs.
- IMU: ICM-20948 over SPI0 with 33 ohm series damping resistors.

## Files

- `DESIGN.md`: design decisions, component calculations, PCB rules and bring-up checklist.
- `WIRING.md`: connector and Teensy 4.1 pin mapping.
- `avtech-agv-carrier/`: KiCad project, schematic and PCB files.
- `output/`: Gerber + drill zip, pick-and-place CSV, BOM, schematic PDF and DRC report.
- `gen/`: generator scripts (`python3 build.py`).
