#include "imu_handler.h"

// ============================================================================
//  IMU Handler — ICM-20948 (SPI)
//  Complementary filter ile yaw hesaplama.
//
//  SPI Protokolü:
//    - Bit7 = 1 → okuma, Bit7 = 0 → yazma
//    - CS LOW → transaction → CS HIGH
//    - Mode 0 (CPOL=0, CPHA=0)
// ============================================================================

// --- ICM-20948 Register Adresleri ---

// Register Bank Select (tüm bank'larda aynı adres)
#define ICM_REG_BANK_SEL        0x7F

// Bank 0 Register'ları
#define ICM_WHO_AM_I            0x00
#define ICM_WHO_AM_I_VAL        0xEA      // ICM-20948 Chip ID

#define ICM_USER_CTRL           0x03
#define ICM_LP_CONFIG           0x05
#define ICM_PWR_MGMT_1          0x06
#define ICM_PWR_MGMT_2          0x07
#define ICM_INT_PIN_CFG         0x0F
#define ICM_INT_ENABLE_1        0x11

// Accelerometer veri register'ları (Bank 0)
#define ICM_ACCEL_XOUT_H        0x2D
#define ICM_ACCEL_XOUT_L        0x2E
#define ICM_ACCEL_YOUT_H        0x2F
#define ICM_ACCEL_YOUT_L        0x30
#define ICM_ACCEL_ZOUT_H        0x31
#define ICM_ACCEL_ZOUT_L        0x32

// Gyroscope veri register'ları (Bank 0)
#define ICM_GYRO_XOUT_H         0x33
#define ICM_GYRO_XOUT_L         0x34
#define ICM_GYRO_YOUT_H         0x35
#define ICM_GYRO_YOUT_L         0x36
#define ICM_GYRO_ZOUT_H         0x37
#define ICM_GYRO_ZOUT_L         0x38

// Bank 2 Register'ları (Gyro ve Accel config)
#define ICM_GYRO_SMPLRT_DIV     0x00      // Bank 2
#define ICM_GYRO_CONFIG_1       0x01      // Bank 2
#define ICM_ACCEL_SMPLRT_DIV_1  0x10      // Bank 2 (MSB)
#define ICM_ACCEL_SMPLRT_DIV_2  0x11      // Bank 2 (LSB)
#define ICM_ACCEL_CONFIG        0x14      // Bank 2

// Gyro ölçek faktörleri (LSB/°/s)
static const float gyro_scale[] = {131.0f, 65.5f, 32.8f, 16.4f};
// Accel ölçek faktörleri (LSB/g)
static const float accel_scale[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};

// --- Dahili Durum ---

static float comp_yaw   = 0.0f;    // Complementary filter yaw
static float comp_pitch = 0.0f;
static float comp_roll  = 0.0f;
static uint32_t last_read_time_ms = 0;
static bool imu_initialized = false;
static float gyro_z_bias = 0.0f;
static uint32_t last_whoami_ms = 0;   // Periyodik WHO_AM_I: SPI kablosu kopunca okuma 0xFF/0x00 doner
static uint8_t  whoami_fails   = 0;   // Ardisik WHO_AM_I hatasi

// SPI ayarları
static SPISettings spi_settings_slow(IMU_SPI_SPEED, MSBFIRST, SPI_MODE0);
static SPISettings spi_settings_fast(IMU_SPI_SPEED_FAST, MSBFIRST, SPI_MODE0);

// Heading PID state
static float h_integral    = 0.0f;
static float h_prev_error  = 0.0f;

// --- SPI Helpers ---

static void spi_write_reg(uint8_t reg, uint8_t val)
{
    IMU_SPI.beginTransaction(spi_settings_slow);
    digitalWriteFast(IMU_CS_PIN, LOW);
    IMU_SPI.transfer(reg & 0x7F);   // Bit7=0 → yazma
    IMU_SPI.transfer(val);
    digitalWriteFast(IMU_CS_PIN, HIGH);
    IMU_SPI.endTransaction();
}

static uint8_t spi_read_reg(uint8_t reg)
{
    IMU_SPI.beginTransaction(spi_settings_fast);
    digitalWriteFast(IMU_CS_PIN, LOW);
    IMU_SPI.transfer(reg | 0x80);   // Bit7=1 → okuma
    uint8_t val = IMU_SPI.transfer(0x00);
    digitalWriteFast(IMU_CS_PIN, HIGH);
    IMU_SPI.endTransaction();
    return val;
}

static int16_t spi_read_reg16(uint8_t reg_h)
{
    IMU_SPI.beginTransaction(spi_settings_fast);
    digitalWriteFast(IMU_CS_PIN, LOW);
    IMU_SPI.transfer(reg_h | 0x80); // Bit7=1 → okuma, burst
    uint8_t msb = IMU_SPI.transfer(0x00);
    uint8_t lsb = IMU_SPI.transfer(0x00);
    digitalWriteFast(IMU_CS_PIN, HIGH);
    IMU_SPI.endTransaction();
    return (int16_t)((msb << 8) | lsb);
}

// 12 byte burst okuma (6 eksen: 3 accel + 3 gyro)
static void spi_read_burst(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    IMU_SPI.beginTransaction(spi_settings_fast);
    digitalWriteFast(IMU_CS_PIN, LOW);
    IMU_SPI.transfer(start_reg | 0x80); // Okuma + auto-increment
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = IMU_SPI.transfer(0x00);
    }
    digitalWriteFast(IMU_CS_PIN, HIGH);
    IMU_SPI.endTransaction();
}

// Bank seçimi (ICM-20948 register'ları 4 bank'a bölünmüş)
static void icm_select_bank(uint8_t bank)
{
    spi_write_reg(ICM_REG_BANK_SEL, (bank & 0x03) << 4);
}

// --- ICM-20948 SPI Init ---

static bool icm20948_init(void)
{
    // SPI başlat
    IMU_SPI.begin();
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWriteFast(IMU_CS_PIN, HIGH); // CS idle = HIGH

    delay(10);

    // Bank 0'a geç
    icm_select_bank(0);

    // WHO_AM_I kontrolü
    uint8_t id = spi_read_reg(ICM_WHO_AM_I);
    if (id != ICM_WHO_AM_I_VAL) {
        Serial.print("[IMU] ICM-20948 not found via SPI, got ID: 0x");
        Serial.println(id, HEX);
        return false;
    }
    Serial.println("[IMU] ICM-20948 detected via SPI (WHO_AM_I = 0xEA)");

    // Device reset
    spi_write_reg(ICM_PWR_MGMT_1, 0x80); // DEVICE_RESET
    delay(100);

    // Reset sonrası CS'yi tekrar ayarla
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWriteFast(IMU_CS_PIN, HIGH);
    delay(10);

    // Bank 0
    icm_select_bank(0);

    // Uyandır: sleep'ten çık, auto clock select
    spi_write_reg(ICM_PWR_MGMT_1, 0x01); // CLKSEL=1 (auto best clock)
    delay(30);

    // Tüm sensörleri etkinleştir
    spi_write_reg(ICM_PWR_MGMT_2, 0x00); // Accel + Gyro hepsi açık
    delay(10);

    // I2C arayüzünü devre dışı bırak (sadece SPI kullanacağız)
    spi_write_reg(ICM_USER_CTRL, 0x10); // I2C_IF_DIS = 1
    delay(5);

    // ── Gyro Konfigürasyonu (Bank 2) ──
    icm_select_bank(2);

    // Gyro sample rate divider (ODR = 1.1kHz / (1 + div))
    spi_write_reg(ICM_GYRO_SMPLRT_DIV, 0x09); // 110 Hz

    // Gyro: seçilen full-scale range + DLPF etkin
    // Bits: [5:3]=GYRO_DLPFCFG(=3, ~51Hz BW), [2:1]=GYRO_FS_SEL, [0]=GYRO_FCHOICE(1)
    uint8_t gyro_cfg = (3 << 3) | ((ICM_GYRO_FS_SEL & 0x03) << 1) | 0x01;
    spi_write_reg(ICM_GYRO_CONFIG_1, gyro_cfg);

    // Accel sample rate divider
    spi_write_reg(ICM_ACCEL_SMPLRT_DIV_1, 0x00); // MSB
    spi_write_reg(ICM_ACCEL_SMPLRT_DIV_2, 0x09); // 110 Hz

    // Accel: seçilen full-scale range + DLPF etkin
    uint8_t accel_cfg = (3 << 3) | ((ICM_ACCEL_FS_SEL & 0x03) << 1) | 0x01;
    spi_write_reg(ICM_ACCEL_CONFIG, accel_cfg);

    // Bank 0'a dön
    icm_select_bank(0);

    // Başlangıç değerleri
    comp_yaw   = 0.0f;
    comp_pitch = 0.0f;
    comp_roll  = 0.0f;

    // --- Gyro Z Bias Calibration ---
    Serial.println("[IMU] Calibrating Gyro Z... Do not move robot!");
    float sum_gz = 0.0f;
    int samples = 100;
    for (int i = 0; i < samples; i++) {
        uint8_t buf[12];
        spi_read_burst(ICM_ACCEL_XOUT_H, buf, 12);
        int16_t raw_gz = (int16_t)((buf[10] << 8) | buf[11]);
        float gz = (float)raw_gz / gyro_scale[ICM_GYRO_FS_SEL];
        sum_gz += gz;
        delay(5);
    }
    gyro_z_bias = sum_gz / (float)samples;
    Serial.print("[IMU] Gyro Z Bias: ");
    Serial.print(gyro_z_bias, 4);
    Serial.println(" deg/s");

    last_read_time_ms = millis();

    Serial.print("[IMU] Gyro range: +/-");
    Serial.print(250 << ICM_GYRO_FS_SEL);
    Serial.print(" deg/s, Accel range: +/-");
    Serial.print(2 << ICM_ACCEL_FS_SEL);
    Serial.println("g");
    Serial.println("[IMU] ICM-20948 SPI initialized (Complementary filter)");

    return true;
}

// --- ICM-20948 SPI Read ---

static bool icm20948_read(IMU_Data_AGV *data)
{
    // Bank 0'da olduğumuzdan emin ol (veri register'ları bank 0'da)
    // NOT: Performans için bank seçimini atla — zaten bank 0'dayız
    // icm_select_bank(0); // gerekirse aç

    // dt hesapla
    uint32_t now = millis();

    // Baglanti kontrolu (2 Hz): kopuk sensor sessizce sifir/cop veri vermesin
    if ((now - last_whoami_ms) >= 500) {
        last_whoami_ms = now;
        uint8_t id = spi_read_reg(ICM_WHO_AM_I);
        if (id != ICM_WHO_AM_I_VAL) {
            Serial.print("[IMU] WHO_AM_I hatasi: 0x");
            Serial.println(id, HEX);
            if (++whoami_fails >= 3) {      // 1.5 s boyunca kopuk → kalici hata
                data->healthy = false;
                return false;
            }
            return true;                    // Tek seferlik parazit: bu ornegi atla
        }
        whoami_fails = 0;
    }

    float dt = (float)(now - last_read_time_ms) / 1000.0f;
    if (dt > 0.5f) dt = 0.01f;
    if (dt < 0.001f) dt = 0.001f;
    last_read_time_ms = now;

    // ── Burst okuma: 12 byte (Accel XYZ + Gyro XYZ) ──
    // Register 0x2D'den başlayarak 12 byte: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L,
    //                                       GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
    uint8_t buf[12];
    spi_read_burst(ICM_ACCEL_XOUT_H, buf, 12);

    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t raw_gy = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gz = (int16_t)((buf[10] << 8) | buf[11]);

    // ── Fiziksel birimlere çevir ──
    float g_scale = gyro_scale[ICM_GYRO_FS_SEL];
    float a_scale = accel_scale[ICM_ACCEL_FS_SEL];

    float gyro_x = (float)raw_gx / g_scale;  // °/s
    float gyro_y = (float)raw_gy / g_scale;
    float gyro_z = ((float)raw_gz / g_scale) - gyro_z_bias;
#if IMU_YAW_INVERT
    gyro_z = -gyro_z;
#endif

    float acc_x = (float)raw_ax / a_scale;    // g
    float acc_y = (float)raw_ay / a_scale;
    float acc_z = (float)raw_az / a_scale;

    // ── Accelerometer'dan açı hesapla ──
    float acc_pitch = atan2f(acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / M_PI;
    float acc_roll  = atan2f(acc_y, sqrtf(acc_x * acc_x + acc_z * acc_z)) * 180.0f / M_PI;

    // ── Complementary Filter ──
    comp_pitch = ICM_COMPLEMENTARY_ALPHA * (comp_pitch + gyro_x * dt)
               + (1.0f - ICM_COMPLEMENTARY_ALPHA) * acc_pitch;
    comp_roll  = ICM_COMPLEMENTARY_ALPHA * (comp_roll  + gyro_y * dt)
               + (1.0f - ICM_COMPLEMENTARY_ALPHA) * acc_roll;

    // Yaw: Sadece gyro entegrasyonu
    comp_yaw += gyro_z * dt;
    while (comp_yaw >= 360.0f) comp_yaw -= 360.0f;
    while (comp_yaw < 0.0f) comp_yaw += 360.0f;

    // ── Struct'ı doldur ──
    data->yaw     = comp_yaw;
    data->pitch   = comp_pitch;
    data->roll    = comp_roll;
    data->gyro_z  = gyro_z;
    data->accel_x = acc_x * 9.81f;  // g → m/s²
    data->accel_y = acc_y * 9.81f;
    data->healthy = true;
    data->last_read_ms = now;

    return true;
}

// --- Public API ---

bool imu_agv_init(void)
{
    imu_initialized = icm20948_init();
    return imu_initialized;
}

bool imu_agv_read(IMU_Data_AGV *data)
{
    if (!imu_initialized) return false;
    return icm20948_read(data);
}

bool imu_agv_is_healthy(void)
{
    return imu_initialized && ((millis() - last_read_time_ms) < 500);
}

float imu_angle_diff(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

float imu_heading_hold(float target_yaw, float current_yaw, float dt)
{
    float error = imu_angle_diff(target_yaw, current_yaw);

    h_integral += error * dt;
    if (h_integral > 80.0f) h_integral = 80.0f;
    if (h_integral < -80.0f) h_integral = -80.0f;

    float derivative = (dt > 0.001f) ? (error - h_prev_error) / dt : 0.0f;
    h_prev_error = error;

    float output = HEADING_KP * error + HEADING_KI * h_integral + HEADING_KD * derivative;

    // Sağ el kuralına göre Z ekseni yukarı bakarken sola dönüş pozitif açıdır.
    // Ancak Hoverboard sürüş sisteminde (Sol = Hız + Steer) pozitif steer aracı SAĞA döndürür.
    // Bu yüzden hedef açımıza (Sola) dönebilmek için steer komutunu ters çeviriyoruz.
    output = -output;

    if (output > (float)MOTOR_STEER_MAX) output = (float)MOTOR_STEER_MAX;
    if (output < -(float)MOTOR_STEER_MAX) output = -(float)MOTOR_STEER_MAX;

    return output;
}

void imu_heading_reset_pid(void)
{
    h_integral = 0.0f;
    h_prev_error = 0.0f;
}

void imu_set_yaw(float absolute_yaw)
{
    comp_yaw = absolute_yaw;
    Serial.print("[IMU] Absolute yaw synchronized: ");
    Serial.println(comp_yaw);
}
