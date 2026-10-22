#ifndef AVTECH_CONFIG_H
#define AVTECH_CONFIG_H

// AVTECH AGV - Pin ve Donanim Konfigurasyon Dosyasi
//
// Elektronik Baglanti Notlari:
// 1. Hoverboard motorlari RX/TX (Serial1) uzerinden baglanacak (FOC USART).
// 2. QTR-8A Cizgi sensorleri A0-A7 analog pinlerine baglanacak.
// 3. HC-SR04 Ultrasonik sensorlerin Trig ve Echo pinlerine dikkat edilmeli.
// 4. ICM-20948 IMU sensoru SPI baglantisi ile baglanacak.
// 5. Yuk Kaldirma (Forklift) Step motoru DM556 surucu ile baglanacak.

// Hoverboard Motor USART (RX1=Pin 0, TX1=Pin 1)
#define HOVER_SERIAL            Serial1   // Tek UART: Speed ve Steer komutu (RX1(0)/TX1(1))
#define HOVER_BAUD              115200
#define HOVER_START_FRAME       0xABCD    // FOC firmware start frame
#define HOVER_SEND_INTERVAL_MS  50        // 20 Hz komut gönderimi
#define HOVER_RECV_TIMEOUT_MS   500       // feedback timeout

// Motor hız limitleri (FOC firmware: -1000 ↔ +1000 aralığı)
#define MOTOR_SPEED_MAX         400       // İleri maksimum
#define MOTOR_SPEED_MIN        -400       // Geri maksimum
#define MOTOR_STEER_MAX         250       // Dönüş maksimum
#define MOTOR_RAMP_STEP         10        // Her döngüde max hız artışı

// Jetson Serial (USB Uzerinden)
#define JETSON_SERIAL           Serial    // USB Serial → Jetson
#define JETSON_BAUD             115200
#define TELEMETRY_INTERVAL_MS   100       // 10 Hz telemetri raporu

// QTR-8A Cizgi Takip Sensoru Pinleri
// A0-A7 arasi sirayla baglanmali (Teensy pin 14-21)
#define LINE_SENSOR_COUNT       8
#define LINE_SENSOR_PINS        {A0, A1, A2, A3, A4, A5, A6, A7}
#define LINE_THRESHOLD          500       // Siyah çizgi eşiği (0-1023)

// 8 sensör ağırlıkları (eşit aralıklı, merkez sıfır)
#define LINE_WEIGHT_0          -3.5f      // En sol  (S0 = A0)
#define LINE_WEIGHT_1          -2.5f      // S1 = A1
#define LINE_WEIGHT_2          -1.5f      // S2 = A2
#define LINE_WEIGHT_3          -0.5f      // S3 = A3
#define LINE_WEIGHT_4           0.5f      // S4 = A4
#define LINE_WEIGHT_5           1.5f      // S5 = A5
#define LINE_WEIGHT_6           2.5f      // S6 = A6
#define LINE_WEIGHT_7           3.5f      // En sağ (S7 = A7)

#define LINE_LOST_TIMEOUT_MS    1000      // Çizgi kayıp zaman aşımı

// Çizgi Takip PID
#define LINE_KP                 1.2f      // QTR-8A daha hassas → düşük P
#define LINE_KI                 0.008f
#define LINE_KD                 0.5f
#define LINE_PID_MAX            MOTOR_STEER_MAX
#define LINE_BASE_SPEED         200       // Çizgi takipte ileri hız

// Ultrasonik Engel Sensorleri (HC-SR04)
// On, Sol ve Sag icin Trig/Echo pinleri:
#define ULTRASONIC_COUNT        3
#define US_FRONT_TRIG           24
#define US_FRONT_ECHO           25
#define US_LEFT_TRIG            26
#define US_LEFT_ECHO            27
#define US_RIGHT_TRIG           28
#define US_RIGHT_ECHO           29

#define US_STOP_DISTANCE_CM     25.0f     // Bu mesafede dur
#define US_SLOW_DISTANCE_CM     50.0f     // Bu mesafede yavaşla
#define US_MAX_RANGE_CM         200.0f    // Maksimum ölçüm mesafesi
#define US_TIMEOUT_US           12000     // pulseIn timeout (~200cm)
#define US_READ_INTERVAL_MS     60        // Okuma aralığı

// IMU - ICM-20948 (SPI Baglantisi)
// SPI0 pinlerine baglanacak:
// SCK  = Pin 13 
// MOSI = Pin 11
// MISO = Pin 12
// CS   = Pin 10
#define IMU_SPI                 SPI       // Teensy 4.1 SPI0
#define IMU_CS_PIN              10        // Chip Select
#define IMU_SPI_SPEED           1000000   // 1 MHz (init sırasında)
#define IMU_SPI_SPEED_FAST      7000000   // 7 MHz (veri okuma sırasında)
#define IMU_READ_INTERVAL_MS    10        // 100 Hz

// ICM-20948 spesifik ayarlar
#define ICM_GYRO_FS_SEL         1         // 0=±250, 1=±500, 2=±1000, 3=±2000 °/s
#define ICM_ACCEL_FS_SEL        1         // 0=±2g, 1=±4g, 2=±8g, 3=±16g
#define ICM_COMPLEMENTARY_ALPHA 0.98f     // Complementary filter katsayısı

// Heading Hold PID
#define HEADING_KP              3.0f
#define HEADING_KI              0.02f
#define HEADING_KD              0.8f
#define HEADING_TOLERANCE_DEG   2.0f
#define HEADING_SETTLE_MS       300

// IMU montaj yonu: arac SAGA donerken (D tusu) GUI'deki yaw AZALMALI.
// Artiyorsa IMU ters takilidir → 1 yapin (yon tutma aksi halde ters calisir; koruma onu kapatir).
#define IMU_YAW_INVERT          0

// Manuel surus — duz gidiste yon tutma (sadece W/S, direksiyon 0 iken IMU yaw'i kilitler)
#define HOLD_ENABLED_DEFAULT    1         // Acilista acik (GUI'den acilip kapatilabilir)
#define HOLD_KP                 4.0f      // steer birimi / derece
#define HOLD_KI                 1.0f      // steer birimi / (derece*s)
#define HOLD_KD                 0.3f      // steer birimi / (derece/s)  (gyro Z ile sonumleme)
#define HOLD_I_MAX              30.0f     // Integral katkisi siniri (steer birimi)
#define HOLD_MAX_STEER          80        // Yon tutma duzeltmesinin mutlak siniri
#define HOLD_PERIOD_MS          20        // 50 Hz duzeltme hesabi
#define HOLD_SETTLE_MS          400       // Direksiyon birakildiktan sonra en gec bu surede kilitle
#define HOLD_SETTLE_GYRO_DPS    3.0f      // Donus hizi bunun altina inince hemen kilitle
#define HOLD_DIVERGE_DEG        25.0f     // Hata bunu asarsa yon tutma bu segment icin kapanir
#define HOLD_MIN_SPEED          15        // Bundan yavas giderken yon tutma yapilmaz

// Forklift Asansoru - Step Motor (DM556 Surucu)
// STEP(PUL), DIR, EN pinleri 3.3V uzerinden baglanacak.
#define STEPPER_STEP_PIN        33        // PUL+ (pulse/step)
#define STEPPER_DIR_PIN         34        // DIR+ (yön)
#define STEPPER_EN_PIN          35        // ENA+ (enable, LOW=aktif)

// Step motor parametreleri
#define STEPPER_STEPS_PER_REV   200       // 1.8° step motor → 200 adım/tur
#define STEPPER_MICROSTEP       8         // DM556 mikro adım (ayar: 1600 pulse/rev)
#define STEPPER_PULSE_US        5         // Minimum pulse genişliği (µs)
#define STEPPER_PULSE_DELAY_US  500       // Adımlar arası bekleme (µs) → hız belirler

// Gripper açma/kapama adım sayıları
#define GRIPPER_OPEN_STEPS      800       // Açma yönünde 800 adım (yarım tur)
#define GRIPPER_CLOSE_STEPS     800       // Kapama yönünde 800 adım
#define GRIPPER_DIR_OPEN        LOW       // Açma/Aşağı yönü (Motor ters takıldığı için LOW oldu)
#define GRIPPER_DIR_CLOSE       HIGH      // Kapama/Yukarı yönü (Motor ters takıldığı için HIGH oldu)
#define GRIPPER_ACTION_MS       2000      // Mekanizma hareket süresi (tahmini)

// Forklift konum takibi (adim sayaci, 0 = en alt)
// Adimlar IntervalTimer kesmesinde uretilir: ana dongu yavaslasa da hareket bozulmaz.
#define FORK_TOP_STEPS          GRIPPER_CLOSE_STEPS  // En ust konum (adim) — strok uzunlugu
#define FORK_JOG_STEPS          100       // GUI/kumanda ince ayar adimi
#define FORK_JOG_MAX_STEPS      2000      // Tek komutta izin verilen max goreli hareket
// GUI "basili tut" hareketi: yavas hiz + keepalive (GUI 100 ms'de bir tekrarlar, kesilirse durur)
#define FORK_SLOW_PULSE_DELAY_US 2000     // Basili tut modunda adim periyodu (500 Hz, tam strok ~1.6 s)
#define FORK_HOLD_TIMEOUT_MS    350       // Bu surede keepalive gelmezse catal durur
#define FORK_DIR_UP             GRIPPER_DIR_CLOSE    // Yukari yonu (DIR pini)
#define FORK_DIR_DOWN           GRIPPER_DIR_OPEN     // Asagi yonu (DIR pini)
// Acilista tam strok asagi surulur (eski davranis): catal alt dayanaga yaslanir, konum = 0.
// 0 yapilirsa catal hareket etmez ve acilistaki konum 0 (alt) kabul edilir.
#define FORK_BOOT_HOME_DOWN     1
// 1 = hareket bitince surucu enable kalir (yuk altinda tutma torku). 0 = eski davranis (EN kapatilir).
#define FORK_HOLD_TORQUE        0

// Guvenlik ve Uyari Donanimlari
// E-Stop butonu basildiginda LOW olacak (INPUT_PULLUP)
#define ESTOP_PIN               36        // Acil durdurma butonu (INPUT_PULLUP)
#define ESTOP_ENABLED           0         // 0 = buton bagli degil, pin yok sayilir. Buton takilinca 1 yapin.
#define BUZZER_PIN              37        // Uyarı buzzer
#define STATUS_LED_PIN          2         // Durum LED'i (Pin 13 artık SPI SCK!)
#define WATCHDOG_TIMEOUT_MS     2000      // Motor komutu yoksa dur
#define MANUAL_CMD_TIMEOUT_MS   500       // MANUAL modda Jetson'dan bu süre komut gelmezse motorlar 0 → IDLE
// Jetson kalp atisi ({"cmd":"hb"}, 5 Hz). Ilk hb geldikten sonra bu sure boyunca hicbir satir
// gelmezse aktif hareket durdurulur. Serial Monitor ile elle test ederken hb gelmedigi icin devre disidir.
#define JETSON_LINK_TIMEOUT_MS  1000

// Sadece kumanda ile manuel sürüş: QTR-8A ve HC-SR04 devre dışı, forklift sonrası
// otonom çizgi takibine geçilmez (IDLE'a dönülür). Otonom mod için 0 yapın.
#define MANUAL_ONLY_MODE        1

// Otonom Gorev Zamanlayicilar (Yazilimsal limitler)
#define NAV_FORWARD_MS          3000      // Düz ileri sürüş süresi
#define NAV_TURN_TIMEOUT_MS     5000      // Dönüş zaman aşımı
#define NAV_MISSION_TIMEOUT_MS  360000    // Toplam görev zaman aşımı (6 dk - Video aşaması için)
#define NAV_PICKUP_APPROACH_MS  2000      // Yük alma yaklaşma süresi
#define NAV_DROP_APPROACH_MS    2000      // Yük bırakma yaklaşma süresi

// ADC Ayarlari
#define ADC_RESOLUTION          10        // 10-bit (0-1023)

// Debug Log Ayarlari
#define DEBUG_ENABLED           1
#define DEBUG_LOG_INTERVAL_MS   500

#endif // AVTECH_CONFIG_H
