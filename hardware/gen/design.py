"""AVTECH AGV tasiyici karti (Rev A) — tek kaynak netlist.

Sematik (make_sch.py) ve PCB (make_pcb.py) bu dosyadan uretilir; bu yuzden
ikisi her zaman ayni baglantilari tasir. Bir baglantiyi degistirmek icin
sadece burayi duzenleyip `python3 hardware/gen/build.py` calistirin.

Tasarim kararlari ve hesaplar: hardware/DESIGN.md
"""

from dataclasses import dataclass, field
from typing import Dict, Optional

# --- Teensy 4.1: fiziksel pin sirasi (ust gorunus, USB yukarida) -------------
# Sol sira yukaridan asagi = footprint pad 1..24, sag sira = pad 25..48.
TEENSY_LEFT = ['GND', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10',
               '11', '12', '3V3', '24', '25', '26', '27', '28', '29', '30',
               '31', '32']
TEENSY_RIGHT = ['VIN', 'GND', '3V3', '23', '22', '21', '20', '19', '18', '17',
                '16', '15', '14', '13', 'GND', '41', '40', '39', '38', '37',
                '36', '35', '34', '33']


def teensy_pad(name: str, occurrence: int = 0) -> str:
    """Teensy pin adindan footprint pad numarasini bulur."""
    hits = [str(i + 1) for i, n in enumerate(TEENSY_LEFT) if n == name]
    hits += [str(i + 25) for i, n in enumerate(TEENSY_RIGHT) if n == name]
    return hits[occurrence]


@dataclass
class Part:
    ref: str
    sym: str                 # KiCad sembol (Lib:Name)
    value: str
    fp: str                  # KiCad footprint (Lib:Name)
    pins: Dict[str, str]     # sembol pin numarasi -> net
    mpn: str = ''
    note: str = ''
    block: str = ''
    dnp: bool = False


PARTS = []


def add(ref, sym, value, fp, pins, mpn='', note='', block=''):
    PARTS.append(Part(ref, sym, value, fp, pins, mpn, note, block))


R0603 = 'Resistor_SMD:R_0603_1608Metric'
C0603 = 'Capacitor_SMD:C_0603_1608Metric'
C0805 = 'Capacitor_SMD:C_0805_2012Metric'
C1206 = 'Capacitor_SMD:C_1206_3216Metric'
C1210 = 'Capacitor_SMD:C_1210_3225Metric'
SOT23 = 'Package_TO_SOT_SMD:SOT-23'
LED0603 = 'LED_SMD:LED_0603_1608Metric'


def R(ref, value, a, b, block, mpn=''):
    add(ref, 'Device:R', value, R0603, {'1': a, '2': b},
        mpn or f'0603 {value} 1%', block=block)


def C(ref, value, a, b, block, fp=C0603, mpn=''):
    add(ref, 'Device:C', value, fp, {'1': a, '2': b}, mpn, block=block)


# =============================================================================
# 1. Guc girisi ve koruma  (24 V aku -> +24V)
# =============================================================================
B = 'power'
add('J1', 'Connector:Screw_Terminal_01x02', '24V AKU',
    'TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal',
    {'1': 'VBAT_IN', '2': 'GND'}, 'Phoenix 1715721', 'Aku girisi (sigortali hattan)', B)
add('F1', 'Device:Fuse', '1A 63V', 'Fuse:Fuse_1206_3216Metric',
    {'1': 'VBAT_IN', '2': 'VBAT_F'}, 'Littelfuse 0466001.NRHF', 'Lojik + kontaktor bobini', B)
add('D1', 'Device:D_Schottky', 'SS16', 'Diode_SMD:D_SMA',
    {'1': '+24V', '2': 'VBAT_F'}, 'SS16 (60V 1A)', 'Ters kutup korumasi', B)
add('D2', 'Device:D_TVS', 'SMBJ30CA', 'Diode_SMD:D_SMB',
    {'1': '+24V', '2': 'GND'}, 'Littelfuse SMBJ30CA', 'Vrwm 30V; motor kaynakli sicramalar', B)
add('C1', 'Device:C_Polarized', '47uF 63V', 'Capacitor_SMD:CP_Elec_8x10',
    {'1': '+24V', '2': 'GND'}, 'Panasonic EEE-FK1J470P', 'Toplu giris kondansatoru', B)
C('C2', '2.2uF 100V', '+24V', 'GND', B, C1210, 'X7R 1210 100V')
C('C3', '2.2uF 100V', '+24V', 'GND', B, C1210, 'X7R 1210 100V')

# =============================================================================
# 2. 24 V -> 5 V buck (LMR16006X, 700 kHz, 0.6 A)
# =============================================================================
B = 'buck'
R('R1', '100k', '+24V', 'BUCK_SHDN', B)
add('U1', 'Regulator_Switching:LMR16006YQ', 'LMR16006XDDCR',
    'Package_TO_SOT_SMD:SOT-23-6',
    {'1': 'BUCK_CB', '2': 'GND', '3': 'BUCK_FB', '4': 'BUCK_SHDN',
     '5': '+24V', '6': 'BUCK_SW'},
    'TI LMR16006XDDCR', 'X versiyonu (700 kHz); sembol Y ile pin uyumlu', B)
C('C4', '100nF 25V', 'BUCK_CB', 'BUCK_SW', B, mpn='X7R 0603 25V')
add('D3', 'Device:D_Schottky', 'SS16', 'Diode_SMD:D_SMA',
    {'1': 'BUCK_SW', '2': 'GND'}, 'SS16 (60V 1A)', 'Buck serbest gecis (catch) diyodu', B)
add('L1', 'Device:L', '22uH', 'Inductor_SMD:L_Bourns_SRN6045TA',
    {'1': 'BUCK_SW', '2': '+5V'}, 'Bourns SRN6045TA-220M', 'Isat >= 1 A gerekli', B)
R('R2', '54.9k', '+5V', 'BUCK_FB', B)
R('R3', '10k', 'BUCK_FB', 'GND', B)
C('C5', '22uF 10V', '+5V', 'GND', B, C1206, 'X7R 1206 10V')
C('C6', '22uF 10V', '+5V', 'GND', B, C1206, 'X7R 1206 10V')
R('R4', '2.2k', '+5V', 'LED_PWR_A', B)
add('D5', 'Device:LED', 'YESIL', LED0603, {'1': 'GND', '2': 'LED_PWR_A'},
    '0603 yesil LED', '+5V var', B)

# =============================================================================
# 3. 5 V -> 3.3 V LDO (sensorler; Teensy kendi 3.3 V'unu kullanir)
# =============================================================================
B = 'ldo'
add('U2', 'Regulator_Linear:AP2112K-3.3', 'AP2112K-3.3', 'Package_TO_SOT_SMD:SOT-23-5',
    {'1': '+5V', '2': 'GND', '3': '+5V', '4': '', '5': '+3V3'},
    'Diodes AP2112K-3.3TRG1', '600 mA; QTR-8A + IMU', B)
C('C7', '1uF', '+5V', 'GND', B, mpn='X7R 0603 16V')
C('C8', '1uF', '+3V3', 'GND', B, mpn='X7R 0603 16V')
C('C9', '10uF', '+3V3', 'GND', B, C0805, 'X5R 0805 10V')
add('D4', 'Device:D_Schottky', 'B5819W', 'Diode_SMD:D_SOD-123',
    {'1': 'TEENSY_VIN', '2': '+5V'}, 'B5819W (40V 1A)',
    'USB 5V ile kart 5V birbirini beslemesin', 'ldo')

# =============================================================================
# 4. Teensy 4.1 (soket)
# =============================================================================
TEENSY_NETS = {
    'VIN': 'TEENSY_VIN', 'GND': 'GND',
    '0': 'T_RX1', '1': 'T_TX1', '2': 'T_LED',
    '10': 'IMU_CS', '11': 'T_MOSI', '12': 'IMU_MISO', '13': 'T_SCK',
    '14': 'QTR1', '15': 'QTR2', '16': 'QTR3', '17': 'QTR4',
    '18': 'QTR5', '19': 'QTR6', '20': 'QTR7', '21': 'QTR8',
    '24': 'T_US_F_TRIG', '25': 'T_US_F_ECHO',
    '26': 'T_US_L_TRIG', '27': 'T_US_L_ECHO',
    '28': 'T_US_R_TRIG', '29': 'T_US_R_ECHO',
    '33': 'T_STEP', '34': 'T_DIR', '35': 'T_ENA',
    '36': 'T_ESTOP_SENSE', '37': 'T_BUZZER', '38': 'T_MOTOR_EN',
}
teensy_pins = {}
for i, name in enumerate(TEENSY_LEFT + TEENSY_RIGHT):
    teensy_pins[str(i + 1)] = TEENSY_NETS.get(name, '')
add('U3', 'avtech:Teensy4.1', 'Teensy 4.1', 'avtech:Teensy4.1_Socket', teensy_pins,
    'PJRC Teensy 4.1 + 2x 1x24 disi header', 'Tum 3.3V pinleri bos; VIN D4 uzerinden', 'mcu')

# =============================================================================
# 5. Hoverboard FOC UART (3.3 V; kablodaki 15 V hattina karsi koruma)
# =============================================================================
B = 'hoverboard'
add('J2', 'Connector_Generic:Conn_01x03', 'HOVERBOARD',
    'Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical',
    {'1': 'GND', '2': 'HB_TX', '3': 'HB_RX'}, 'JST B3B-XH-A',
    '15V pini bilerek yok', B)
R('R5', '1k', 'T_TX1', 'HB_TX', B)
R('R6', '1k', 'HB_RX', 'T_RX1', B)
add('D6', 'Diode:BAT54S', 'BAT54S', SOT23, {'1': 'GND', '2': '+3V3', '3': 'T_RX1'},
    'BAT54S', 'RX kiskaci', B)
add('D7', 'Diode:BAT54S', 'BAT54S', SOT23, {'1': 'GND', '2': '+3V3', '3': 'T_TX1'},
    'BAT54S', 'TX kiskaci', B)

# =============================================================================
# 6. IMU (ICM-20948 karti, SPI0)
# =============================================================================
B = 'imu'
add('J3', 'Connector_Generic:Conn_01x06', 'IMU SPI',
    'Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical',
    {'1': '+3V3', '2': 'GND', '3': 'IMU_SCK', '4': 'IMU_MOSI', '5': 'IMU_MISO',
     '6': 'IMU_CS'}, 'JST B6B-XH-A', '', B)
R('R7', '33', 'T_SCK', 'IMU_SCK', B)
R('R8', '33', 'T_MOSI', 'IMU_MOSI', B)
C('C10', '100nF', '+3V3', 'GND', B, mpn='X7R 0603 16V')

# =============================================================================
# 7. QTR-8A cizgi sensoru (3.3 V ile beslenir -> cikislar ADC sinirinda)
# =============================================================================
B = 'qtr'
qtr = {'1': '+3V3', '2': 'GND', '3': '+3V3'}
for k in range(8):
    qtr[str(k + 4)] = f'QTR{k + 1}'
add('J4', 'Connector_Generic:Conn_01x11', 'QTR-8A',
    'Connector_JST:JST_XH_B11B-XH-A_1x11_P2.50mm_Vertical', qtr,
    'JST B11B-XH-A', 'Pin 3 = LEDON (hep acik)', B)
C('C11', '100nF', '+3V3', 'GND', B, mpn='X7R 0603 16V')

# =============================================================================
# 8. HC-SR04 x3 (5 V besleme; ECHO 5 V -> bolucu ile 3.0 V)
# =============================================================================
B = 'ultrasonic'
for i, (side, jref, rt, rtop, rbot) in enumerate([
        ('F', 'J5', 'R9', 'R12', 'R13'),
        ('L', 'J6', 'R10', 'R14', 'R15'),
        ('R', 'J7', 'R11', 'R16', 'R17')]):
    name = {'F': 'ON', 'L': 'SOL', 'R': 'SAG'}[side]
    add(jref, 'Connector_Generic:Conn_01x04', f'HC-SR04 {name}',
        'Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical',
        {'1': '+5V', '2': f'US_{side}_TRIG', '3': f'US_{side}_ECHO5', '4': 'GND'},
        'JST B4B-XH-A', '', B)
    R(rt, '100', f'T_US_{side}_TRIG', f'US_{side}_TRIG', B)
    R(rtop, '2.2k', f'US_{side}_ECHO5', f'T_US_{side}_ECHO', B)
    R(rbot, '3.3k', f'T_US_{side}_ECHO', 'GND', B)

# =============================================================================
# 9. DM556 step surucu (ortak anot 5 V; Teensy alcak taraf anahtarlar)
# =============================================================================
B = 'stepper'
add('J8', 'Connector_Generic:Conn_01x06', 'DM556',
    'Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical',
    {'1': '+5V', '2': 'STEP_N', '3': '+5V', '4': 'DIR_N', '5': '+5V', '6': 'ENA_N'},
    'JST B6B-XH-A', 'PUL+ PUL- DIR+ DIR- ENA+ ENA-', B)
for q, rg, rp, tnet, dnet in [('Q1', 'R18', 'R21', 'T_STEP', 'STEP_N'),
                              ('Q2', 'R19', 'R22', 'T_DIR', 'DIR_N'),
                              ('Q3', 'R20', 'R23', 'T_ENA', 'ENA_N')]:
    g = f'{q}_G'
    add(q, 'Transistor_FET:AO3400A', 'AO3400A', SOT23,
        {'1': g, '2': 'GND', '3': dnet}, 'AOS AO3400A', '', B)
    R(rg, '100', tnet, g, B)
    R(rp, '100k', g, 'GND', B)

# =============================================================================
# 10. Buzzer ve durum LED'i
# =============================================================================
B = 'ui'
add('BZ1', 'Device:Buzzer', '5V aktif', 'Buzzer_Beeper:Buzzer_12x9.5RM7.6',
    {'1': '+5V', '2': 'BUZ_N'}, '12 mm 5V aktif buzzer', '', B)
add('Q4', 'Transistor_FET:AO3400A', 'AO3400A', SOT23,
    {'1': 'Q4_G', '2': 'GND', '3': 'BUZ_N'}, 'AOS AO3400A', '', B)
R('R24', '100', 'T_BUZZER', 'Q4_G', B)
R('R25', '100k', 'Q4_G', 'GND', B)
add('D8', 'Device:D', '1N4148W', 'Diode_SMD:D_SOD-323',
    {'1': '+5V', '2': 'BUZ_N'}, '1N4148WS', 'Buzzer serbest gecis', B)
R('R26', '330', 'T_LED', 'LED_STAT_A', B)
add('D9', 'Device:LED', 'MAVI', LED0603, {'1': 'GND', '2': 'LED_STAT_A'},
    '0603 mavi LED', 'Durum (pin 2)', B)

# =============================================================================
# 11. Donanimsal acil stop + motor gucu kontaktoru
#   +24V -> [E-stop NC buton] -> ESTOP_RET -> [kontaktor bobini] -> COIL_N -> Q5 -> GND
#   Buton basilinca bobin yaziliminin durumundan bagimsiz olarak duser.
#   Teensy MOTOR_EN ile bobini ayrica birakabilir; Teensy resetteyken Q5 kapalidir.
# =============================================================================
B = 'estop'
add('J9', 'Connector_Generic:Conn_01x02', 'E-STOP (NC)',
    'Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical',
    {'1': '+24V', '2': 'ESTOP_RET'}, 'JST B2B-XH-A', 'Normalde kapali buton', B)
add('J10', 'Connector_Generic:Conn_01x02', 'KONTAKTOR',
    'Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical',
    {'1': 'ESTOP_RET', '2': 'COIL_N'}, 'JST B2B-XH-A', '24V bobin, <= 0.4 A', B)
add('Q5', 'Transistor_BJT:BCP56', 'BCP56-16', 'Package_TO_SOT_SMD:SOT-223-3_TabPin2',
    {'1': 'Q5_B', '2': 'COIL_N', '3': 'GND', '4': 'COIL_N'},
    'Nexperia BCP56-16', '80V 1A; hFE 100-250', B)
R('R27', '470', 'T_MOTOR_EN', 'Q5_B', B)
R('R28', '10k', 'Q5_B', 'GND', B)
add('D10', 'Device:D_Schottky', 'SS16', 'Diode_SMD:D_SMA',
    {'1': 'ESTOP_RET', '2': 'COIL_N'}, 'SS16 (60V 1A)', 'Bobin serbest gecis diyodu', B)
R('R29', '10k', 'ESTOP_RET', 'LED_MOT_A', B)
add('D11', 'Device:LED', 'KIRMIZI', LED0603, {'1': 'COIL_N', '2': 'LED_MOT_A'},
    '0603 kirmizi LED', 'Motor gucu acik', B)
R('R30', '47k', 'ESTOP_RET', 'ESTOP_DIV', B)
R('R31', '10k', 'ESTOP_DIV', 'GND', B)
add('D12', 'Device:D_Zener', 'BZT52C3V3', 'Diode_SMD:D_SOD-123',
    {'1': 'ESTOP_DIV', '2': 'GND'}, 'BZT52C3V3', '3.3 V kiskac', B)
R('R32', '1k', 'ESTOP_DIV', 'T_ESTOP_SENSE', B)

# =============================================================================
# 12. Test noktalari, montaj delikleri, ERC guc bayraklari
# =============================================================================
B = 'misc'
for tp, net in [('TP1', '+24V'), ('TP2', '+5V'), ('TP3', '+3V3'), ('TP4', 'GND')]:
    add(tp, 'Connector:TestPoint', net, 'TestPoint:TestPoint_Pad_D1.5mm',
        {'1': net}, '', '', B)
for h in ['H1', 'H2', 'H3', 'H4']:
    add(h, 'Mechanical:MountingHole', 'M3', 'MountingHole:MountingHole_3.2mm_M3',
        {}, '', '', B)

# PWR_FLAG: sadece sematikte (ERC icin), PCB'de yok
PWR_FLAGS = ['+24V', 'GND', '+5V', 'TEENSY_VIN']


def nets():
    out = {}
    for p in PARTS:
        for pin, n in p.pins.items():
            if n:
                out.setdefault(n, []).append((p.ref, pin))
    return out


if __name__ == '__main__':
    ns = nets()
    print(f'{len(PARTS)} parca, {len(ns)} net')
    for n, conns in sorted(ns.items()):
        if len(conns) < 2:
            print('  TEK BAGLANTILI NET:', n, conns)
