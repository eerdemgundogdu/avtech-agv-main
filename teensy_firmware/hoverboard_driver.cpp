#include "hoverboard_driver.h"

// ============================================================================
//  Hoverboard USART Driver — FOC SPD_MODE 
//
//  Önemli değişiklikler (orijinalden farklar):
//  1. Word-level XOR checksum (FOC firmware ile uyumlu)
//  2. Yazılımsal hız ramping (ani hız geçişi yok)
//  3. Her iki motor hız okuması (speedR + speedL)
//  4. Odometri kümülatif yol hesabı
//  5. Checksum hata sayacı ve güvenilirlik metrikleri
//  6. Güvenli durdurma mekanizması
// ============================================================================

static MotorStatus   motor_left   = {};
static MotorStatus   motor_right  = {};
static OdometryData  odom_left    = {};
static OdometryData  odom_right   = {};
static RampState     ramp_speed   = {};
static RampState     ramp_steer   = {};

// Feedback parse buffer
static uint8_t   recv_buf[sizeof(HoverFeedback)];
static uint16_t  recv_idx  = 0;

// Odometri sabitleri
// Hoverboard tekerlek çapı ~16.5cm → çevre = 0.5184m
// FOC firmware speed ölçümü: her Hall sensor tick başına 1 birim
// Tipik hoverboard: 15 mıknatıs çifti → 1 devir = 90 electrical revolution
// Basitleştirilmiş: speed_meas ≈ RPM * scale_factor
static const float WHEEL_CIRCUMFERENCE_M = 0.165f * 3.14159f;  // ~0.5184m
static const float SPEED_TO_RPS = 1.0f / 60.0f;  // RPM → RPS
static const float ODOM_VELOCITY_FILTER = 0.85f;  // Düşük geçiş filtre katsayısı

// FOC firmware checksum'ı 16-bit word bazında XOR yapıyor.
// Komut: start ^ steer ^ speed
// Feedback: start ^ cmd1 ^ cmd2 ^ speedR ^ speedL ^ batV ^ boardT ^ cmdLed
static uint16_t calc_checksum_words(const uint16_t *words, uint16_t word_count)
{
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < word_count; i++) {
        checksum ^= words[i];
    }
    return checksum;
}

static void send_command_raw(HardwareSerial &port, int16_t steer, int16_t speed)
{
    HoverCommand cmd;
    cmd.start = HOVER_START_FRAME;
    cmd.steer = steer;
    cmd.speed = speed;
    // Word-level XOR: start(uint16) ^ steer(cast to uint16) ^ speed(cast to uint16)
    cmd.checksum = cmd.start ^ (uint16_t)cmd.steer ^ (uint16_t)cmd.speed;
    port.write((uint8_t *)&cmd, sizeof(cmd));
}

static bool parse_feedback(HardwareSerial &port, uint8_t *buf, uint16_t &idx)
{
    bool got_new = false;

    while (port.available()) {
        uint8_t byte_in = port.read();

        // Start frame tespiti (little-endian: 0xCD sonra 0xAB)
        if (idx < 2) {
            if (idx == 0 && byte_in == (HOVER_START_FRAME & 0xFF)) {
                buf[idx++] = byte_in;
            } else if (idx == 1 && byte_in == ((HOVER_START_FRAME >> 8) & 0xFF)) {
                buf[idx++] = byte_in;
            } else {
                // İlk byte 0xCD ile eşleşmiyorsa tamamen sıfırla,
                // ama gelen byte 0xCD ise yeni paket başlangıcı olabilir
                if (byte_in == (HOVER_START_FRAME & 0xFF)) {
                    buf[0] = byte_in;
                    idx = 1;
                } else {
                    idx = 0;
                }
            }
        } else {
            buf[idx++] = byte_in;

            if (idx >= sizeof(HoverFeedback)) {
                HoverFeedback *fb = (HoverFeedback *)buf;

                // Word-level XOR checksum doğrulama
                // Checksum hariç tüm 16-bit word'lerin XOR'u
                uint16_t num_words = (sizeof(HoverFeedback) - sizeof(uint16_t)) / sizeof(uint16_t);
                uint16_t expected = calc_checksum_words((uint16_t *)buf, num_words);

                if (fb->checksum == expected) {
                    // Sol Motor
                    motor_left.connected     = true;
                    motor_left.cmd_echo      = fb->cmd1;
                    motor_left.speedL_raw    = fb->speedL_meas;
                    motor_left.speedR_raw    = fb->speedR_meas;
                    motor_left.speed_meas    = fb->speedL_meas;
                    motor_left.bat_voltage   = fb->batVoltage;
                    motor_left.board_temp    = fb->boardTemp;
                    motor_left.last_recv_ms  = millis();
                    motor_left.recv_count++;

                    // Sağ Motor
                    motor_right.connected    = true;
                    motor_right.cmd_echo     = fb->cmd2;
                    motor_right.speedL_raw   = fb->speedL_meas;
                    motor_right.speedR_raw   = fb->speedR_meas;
                    motor_right.speed_meas   = fb->speedR_meas;
                    motor_right.bat_voltage  = fb->batVoltage;
                    motor_right.board_temp   = fb->boardTemp;
                    motor_right.last_recv_ms = millis();
                    motor_right.recv_count++;

                    got_new = true;
                } else {
                    motor_left.checksum_errors++;
                    motor_right.checksum_errors++;
                }

                idx = 0; // Sonraki paket
            }
        }
    }

    // Bağlantı timeout kontrolü
    if (motor_left.connected && (millis() - motor_left.last_recv_ms) > HOVER_RECV_TIMEOUT_MS) {
        motor_left.connected = false;
        motor_right.connected = false;
    }

    return got_new;
}

static int16_t apply_ramp(RampState &ramp, uint32_t now)
{
    if (ramp.current == ramp.target) {
        return ramp.current;
    }

    ramp.last_ramp_ms = now;

    int16_t diff = ramp.target - ramp.current;

    // Durdurma durumunda daha hızlı ramp (güvenlik)
    int16_t step = MOTOR_RAMP_STEP;
    if (ramp.target == 0) {
        step = MOTOR_RAMP_STEP * 2; // 2x hızlı durdurma
    }

    if (diff > step) {
        ramp.current += step;
    } else if (diff < -step) {
        ramp.current -= step;
    } else {
        ramp.current = ramp.target;
    }

    return ramp.current;
}

void hoverboard_init(void)
{
    HOVER_SERIAL.begin(HOVER_BAUD);

    // Motor durumları sıfırla
    memset(&motor_left,  0, sizeof(MotorStatus));
    memset(&motor_right, 0, sizeof(MotorStatus));
    memset(&odom_left,   0, sizeof(OdometryData));
    memset(&odom_right,  0, sizeof(OdometryData));
    memset(&ramp_speed,  0, sizeof(RampState));
    memset(&ramp_steer,  0, sizeof(RampState));
    recv_idx = 0;

    // Başlangıçta motorları durdur
    send_command_raw(HOVER_SERIAL, 0, 0);

    Serial.println("[HOVER] SPD_MODE USART driver initialized (Single UART)");
    Serial.println("[HOVER]   Port: Serial1 (RX0/TX1)");
    Serial.println("[HOVER]   Ramping: enabled, step=" + String(MOTOR_RAMP_STEP));
}

void hoverboard_send(int16_t speed, int16_t steer)
{
    ramp_speed.target = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    ramp_steer.target = constrain(steer, -MOTOR_STEER_MAX, MOTOR_STEER_MAX);
}

void hoverboard_send_direct(int16_t speed, int16_t steer)
{
    int16_t sp = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    int16_t st = constrain(steer, -MOTOR_STEER_MAX, MOTOR_STEER_MAX);

    send_command_raw(HOVER_SERIAL, st, sp);

    ramp_speed.current = sp;
    ramp_speed.target  = sp;
    ramp_steer.current = st;
    ramp_steer.target  = st;
}

void hoverboard_stop(void)
{
    ramp_speed.current = 0; ramp_speed.target = 0;
    ramp_steer.current = 0; ramp_steer.target = 0;
    send_command_raw(HOVER_SERIAL, 0, 0);
    send_command_raw(HOVER_SERIAL, 0, 0);
}

void hoverboard_soft_stop(void)
{
    ramp_speed.target = 0;
    ramp_steer.target = 0;
}

void hoverboard_receive(void)
{
    parse_feedback(HOVER_SERIAL, recv_buf, recv_idx);
}

void hoverboard_update_ramp(void)
{
    uint32_t now = millis();
    int16_t sp = apply_ramp(ramp_speed, now);
    int16_t st = apply_ramp(ramp_steer, now);
    send_command_raw(HOVER_SERIAL, st, sp);
}

void hoverboard_update_odometry(float dt)
{
    if (dt <= 0.0f || dt > 1.0f) return;

    // Sol motor odometri
    {
        // speed_meas → RPM (yaklaşık)
        float rpm = (float)motor_left.speed_meas;
        float rps = rpm * SPEED_TO_RPS;
        float velocity = rps * WHEEL_CIRCUMFERENCE_M; // m/s

        // Düşük geçiş filtre
        odom_left.velocity_filtered = ODOM_VELOCITY_FILTER * odom_left.velocity_filtered
                                    + (1.0f - ODOM_VELOCITY_FILTER) * velocity;
        odom_left.velocity_mps = velocity;
        odom_left.distance_m += fabsf(velocity) * dt;
        odom_left.last_update_ms = millis();
    }

    // Sağ motor odometri
    {
        float rpm = (float)motor_right.speed_meas;
        float rps = rpm * SPEED_TO_RPS;
        float velocity = rps * WHEEL_CIRCUMFERENCE_M;

        odom_right.velocity_filtered = ODOM_VELOCITY_FILTER * odom_right.velocity_filtered
                                     + (1.0f - ODOM_VELOCITY_FILTER) * velocity;
        odom_right.velocity_mps = velocity;
        odom_right.distance_m += fabsf(velocity) * dt;
        odom_right.last_update_ms = millis();
    }
}

bool hoverboard_left_connected(void)  { return motor_left.connected; }
bool hoverboard_right_connected(void) { return motor_right.connected; }
bool hoverboard_any_connected(void)   { return motor_left.connected || motor_right.connected; }

const MotorStatus*  hoverboard_get_left(void)  { return &motor_left; }
const MotorStatus*  hoverboard_get_right(void) { return &motor_right; }
const OdometryData* hoverboard_get_odometry_left(void)  { return &odom_left; }
const OdometryData* hoverboard_get_odometry_right(void) { return &odom_right; }

float hoverboard_get_battery_voltage(void)
{
    // Batarya gerilimi: değer x100 (ör: 2400 = 24.00V)
    // Bağlı olan motorlardan oku
    if (motor_left.connected && motor_right.connected) {
        return ((float)motor_left.bat_voltage + (float)motor_right.bat_voltage) / 200.0f;
    }
    if (motor_left.connected)  return (float)motor_left.bat_voltage / 100.0f;
    if (motor_right.connected) return (float)motor_right.bat_voltage / 100.0f;
    return 0.0f;
}

float hoverboard_get_avg_speed(void)
{
    // Ortalama RPM
    return ((float)motor_left.speed_meas + (float)motor_right.speed_meas) / 2.0f;
}

float hoverboard_get_avg_velocity_mps(void)
{
    return (odom_left.velocity_filtered + odom_right.velocity_filtered) / 2.0f;
}

float hoverboard_get_total_distance(void)
{
    return (odom_left.distance_m + odom_right.distance_m) / 2.0f;
}

float hoverboard_get_speed_error_left(void)
{
    if (ramp_speed.current == 0) return 0.0f;
    return fabsf((float)(ramp_speed.current - ramp_steer.current) - (float)motor_left.speed_meas)
           / fabsf((float)ramp_speed.current) * 100.0f;
}

float hoverboard_get_speed_error_right(void)
{
    if (ramp_speed.current == 0) return 0.0f;
    return fabsf((float)(ramp_speed.current + ramp_steer.current) - (float)motor_right.speed_meas)
           / fabsf((float)ramp_speed.current) * 100.0f;
}

bool hoverboard_speed_within_tolerance(float tolerance_pct)
{
    // Her iki motor da tolerans içinde mi?
    return hoverboard_get_speed_error_left() < tolerance_pct
        && hoverboard_get_speed_error_right() < tolerance_pct;
}
