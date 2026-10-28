# Kablolama — AVTECH AGV taşıyıcı kart Rev A

Her konnektörün pin pin nereye gittiği. Pin 1, kart üzerinde kare pedli olan ve silkscreen'de üçgenle işaretli pindir. JST XH konnektörler kilitlidir, ters takılamaz.

> **Rev A henüz üretilmedi ve test edilmedi.** İlk enerji vermeden önce [DESIGN.md](DESIGN.md) içindeki devreye alma adımlarını uygulayın.

## Güç ve güvenlik

### J1 — 24 V akü (vidalı klemens, 5.08 mm)

| Pin | Net | Bağlantı |
|----|-----|----------|
| 1 | `VBAT_IN` | Akü + (ana sigortadan **sonra**, kontaktörden **önce**) |
| 2 | `GND` | Akü − |

Kartın kendi sigortası (F1, 1 A) yalnızca lojik ve kontaktör bobinini korur. Hoverboard motor akımı karttan geçmez; motor hattı için ayrı bir ana sigorta gerekir.

### J9 — Acil stop butonu (JST XH 2p)

| Pin | Net | Bağlantı |
|----|-----|----------|
| 1 | `+24V` | E-stop butonunun **NC** (normalde kapalı) kontağı, uç 1 |
| 2 | `ESTOP_RET` | NC kontak, uç 2 |

Butonun NC kontağı kullanılmalı. Kablo koparsa veya konnektör çıkarsa döngü açılır ve motor gücü kesilir (fail-safe).

### J10 — Motor gücü kontaktörü bobini (JST XH 2p)

| Pin | Net | Bağlantı |
|----|-----|----------|
| 1 | `ESTOP_RET` | Bobin + (E-stop'tan gelen 24 V) |
| 2 | `COIL_N` | Bobin − (kartta Q5 ile GND'ye anahtarlanır) |

Bobin 24 V DC ve **en fazla 0.4 A** olmalı. Kontaktörün ana kontakları (NO) akü ile iki hoverboard kartı arasına bağlanır ve motor akımına uygun seçilmelidir. Serbest geçiş diyodu (D10) kartta; bobine ayrıca diyot gerekmez.

## Hareket

### J2 — Hoverboard FOC UART (JST XH 3p)

| Pin | Net | Hoverboard tarafı |
|----|-----|-------------------|
| 1 | `GND` | GND |
| 2 | `HB_TX` | Hoverboard **RX** (Teensy TX1, 1 kΩ seri) |
| 3 | `HB_RX` | Hoverboard **TX** (Teensy RX1, 1 kΩ seri + kıskaç) |

Hoverboard kablosundaki **15 V pini bağlanmaz**. Konnektörde bu pin bilerek yok; kablodaki 15 V teli izole edilmelidir.

### J8 — DM556 step sürücü (JST XH 6p)

Bağlantı ortak anot tipindedir: + uçlar 5 V'a gider, − uçlar kartta anahtarlanır.

| Pin | Net | DM556 |
|----|-----|-------|
| 1 | `+5V` | PUL+ |
| 2 | `STEP_N` | PUL− |
| 3 | `+5V` | DIR+ |
| 4 | `DIR_N` | DIR− |
| 5 | `+5V` | ENA+ |
| 6 | `ENA_N` | ENA− |

Mantık seviyesi mevcut kablolamayla aynıdır: Teensy pini HIGH olduğunda optokuplör iletime geçer. Firmware'de değişiklik gerekmez.

## Sensörler

### J3 — ICM-20948 IMU kartı, SPI (JST XH 6p)

| Pin | Net | IMU kartı |
|----|-----|-----------|
| 1 | `+3V3` | 3V3 / VIN |
| 2 | `GND` | GND |
| 3 | `IMU_SCK` | SCL/SCK |
| 4 | `IMU_MOSI` | SDA/SDI |
| 5 | `IMU_MISO` | AD0/SDO |
| 6 | `IMU_CS` | CS |

SPI kablosu kısa tutulmalı (< 20 cm); SCK ve MOSI hatlarında 33 Ω seri sönümleme dirençleri var.

### J4 — QTR-8A çizgi sensörü (JST XH 11p)

| Pin | Net | QTR-8A |
|----|-----|--------|
| 1 | `+3V3` | VCC |
| 2 | `GND` | GND |
| 3 | `+3V3` | LEDON (IR LED'ler hep açık) |
| 4…11 | `QTR1`…`QTR8` | OUT1…OUT8 → Teensy A0…A7 (pin 14…21) |

QTR-8A bu kartta **3.3 V ile beslenir**; böylece analog çıkışlar ADC'nin 3.3 V sınırını aşmaz. Besleme gerilimi değiştiği için `LINE_THRESHOLD` yeniden kalibre edilmelidir.

### J5 / J6 / J7 — HC-SR04 ön / sol / sağ (JST XH 4p)

| Pin | Net | HC-SR04 |
|----|-----|---------|
| 1 | `+5V` | VCC |
| 2 | `US_x_TRIG` | Trig (Teensy 24 / 26 / 28, 100 Ω seri) |
| 3 | `US_x_ECHO5` | Echo (5 V) → 2.2 k / 3.3 k bölücü → 3.0 V → Teensy 25 / 27 / 29 |
| 4 | `GND` | GND |

## Teensy 4.1

Teensy, iki adet 1×24 dişi header'a takılır; USB konnektörü kartın üst kenarına bakar. USB kablosu Jetson'a doğrudan bağlanır, kart üzerinden geçmez.

| Teensy pini | Net | Görev |
|-------------|-----|-------|
| VIN | `TEENSY_VIN` | Kart 5 V'u, D4 Schottky üzerinden |
| 0 / 1 | `T_RX1` / `T_TX1` | Hoverboard UART |
| 2 | `T_LED` | Durum LED'i (D9, mavi) |
| 10 / 11 / 12 / 13 | `IMU_CS` / `T_MOSI` / `IMU_MISO` / `T_SCK` | IMU SPI0 |
| 14…21 | `QTR1`…`QTR8` | Çizgi sensörü analog |
| 24…29 | `T_US_*` | HC-SR04 trig/echo |
| 33 / 34 / 35 | `T_STEP` / `T_DIR` / `T_ENA` | DM556 |
| 36 | `T_ESTOP_SENSE` | E-stop döngü algılama (normal HIGH, basılı LOW) |
| 37 | `T_BUZZER` | Buzzer (Q4) |
| 38 | `T_MOTOR_EN` | Kontaktör bobini (Q5) — `CARRIER_BOARD_REV=1` |

Teensy'nin 3.3 V pinleri kartta kullanılmaz; sensörler kartın kendi 3.3 V LDO'sundan (U2) beslenir.

## Kart üstü göstergeler

| LED | Renk | Anlamı |
|-----|------|--------|
| D5 | Yeşil | Kart 5 V'u var |
| D9 | Mavi | Firmware durum LED'i (pin 2) |
| D11 | Kırmızı | Kontaktör bobini enerjili (motor gücü açık) |

## Test noktaları

| TP | Net | Beklenen |
|----|-----|----------|
| TP1 | `+24V` | Akü − ~0.4 V (D1 düşümü) |
| TP2 | `+5V` | 4.85 … 5.08 V |
| TP3 | `+3V3` | 3.23 … 3.37 V |
| TP4 | `GND` | 0 V referans |
