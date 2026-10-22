#include "line_follower.h"

// ============================================================================
//  8-Kanal QTR-8A Çizgi Takip — Ağırlıklı Ortalama + PID
//  Pololu QTR-8A analog reflektans sensör dizisi.
// ============================================================================

static const int line_pins[LINE_SENSOR_COUNT] = LINE_SENSOR_PINS;
static const float sensor_weights[LINE_SENSOR_COUNT] = {
    LINE_WEIGHT_0,   // En sol   (-3.5)
    LINE_WEIGHT_1,   //          (-2.5)
    LINE_WEIGHT_2,   //          (-1.5)
    LINE_WEIGHT_3,   // Sol-mrk  (-0.5)
    LINE_WEIGHT_4,   // Sağ-mrk  (+0.5)
    LINE_WEIGHT_5,   //          (+1.5)
    LINE_WEIGHT_6,   //          (+2.5)
    LINE_WEIGHT_7    // En sağ   (+3.5)
};

static LineData data;
static float pid_integral    = 0.0f;
static float pid_prev_error  = 0.0f;
static uint32_t pid_prev_ms  = 0;
static LineState last_known_dir = LINE_STATE_ON_LINE;

// Kalibrasyon değişkenleri
static int sensor_min[LINE_SENSOR_COUNT];
static int sensor_max[LINE_SENSOR_COUNT];

// --- Initialization ---

void line_init(void)
{
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        pinMode(line_pins[i], INPUT);
        sensor_min[i] = 1023;
        sensor_max[i] = 0;
    }

    memset(&data, 0, sizeof(data));
    data.state       = LINE_STATE_LOST_ALL;
    data.last_seen_ms = 0;
    pid_integral     = 0.0f;
    pid_prev_error   = 0.0f;
    pid_prev_ms      = millis();

    Serial.println("[LINE] QTR-8A 8-channel analog line sensor initialized");
    Serial.print("[LINE] Pins: ");
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        Serial.print("A");
        Serial.print(i);
        if (i < LINE_SENSOR_COUNT - 1) Serial.print(", ");
    }
    Serial.println();
}

// --- Update ---

void line_update(void)
{
    uint32_t now = millis();
    float dt = (pid_prev_ms == 0) ? 0.01f : (float)(now - pid_prev_ms) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    pid_prev_ms = now;

    // 1) Sensörleri oku ve dinamik eşik hesapla
    data.active_count = 0;
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        int raw = analogRead(line_pins[i]);
        data.raw[i] = raw;

        // Dinamik kalibrasyon takibi
        if (raw < sensor_min[i]) sensor_min[i] = raw;
        if (raw > sensor_max[i]) sensor_max[i] = raw;

        // Normalization (0-1000 aralığına çekme)
        int range = sensor_max[i] - sensor_min[i];
        if (range < 1) range = 1; // Sıfıra bölmeyi engelle

        int normalized = (raw - sensor_min[i]) * 1000 / range;
        if (normalized < 0) normalized = 0;
        if (normalized > 1000) normalized = 1000;

        // Kontrast çok düşükse (örn. henüz kalibre edilmediyse) ham veriyi ve config eşiğini kullan
        int threshold = 500;
        if (range < 200) {
            data.raw[i] = raw;
            threshold = LINE_THRESHOLD;
        } else {
            data.raw[i] = normalized;
        }

        data.active[i] = (data.raw[i] > threshold);
        if (data.active[i]) data.active_count++;

        // Min/Max değerleri zamanla ortaya doğru kaydır (decay) böylece ortam ışığı değiştiğinde adapte olur
        if (now % 100 == 0) { // Her ~100ms'de bir ufak daralma
            if (sensor_min[i] < 1023) sensor_min[i]++;
            if (sensor_max[i] > 0) sensor_max[i]--;
        }
    }

    // 2) Ağırlıklı ortalama ile pozisyon hesapla (Durum belirlemeden ÖNCE yapılmalı)
    if (data.active_count > 0) {
        float weighted_sum = 0.0f;
        float total_weight = 0.0f;
        for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
            if (data.active[i]) {
                weighted_sum += sensor_weights[i] * (float)data.raw[i];
                total_weight += (float)data.raw[i];
            }
        }
        if (total_weight > 0.01f) {
            data.position = weighted_sum / total_weight;
        }
    }
    // Kayıp durumda pozisyonu koruyoruz (son bilinen)

    // 3) Durum belirle (Artık taze pozisyon verisini kullanıyor)
    if (data.active_count == 0) {
        // Çizgi hiç görünmüyor
        if (data.state == LINE_STATE_ON_LINE || data.state == LINE_STATE_CROSS) {
            // Hangi taraftan kayboldu?
            if (data.position < -0.5f) {
                data.state = LINE_STATE_LOST_LEFT;
                last_known_dir = LINE_STATE_LOST_LEFT;
            } else if (data.position > 0.5f) {
                data.state = LINE_STATE_LOST_RIGHT;
                last_known_dir = LINE_STATE_LOST_RIGHT;
            } else {
                data.state = LINE_STATE_LOST_ALL;
            }
        }
        // Eğer zaten kayıp durumundaysa timeout kontrol
        if ((now - data.last_seen_ms) > LINE_LOST_TIMEOUT_MS && data.last_seen_ms > 0) {
            data.state = LINE_STATE_LOST_ALL;
        }
    } else if (data.active_count >= 4) {
        // 8 sensörden 4 veya daha fazlası aktif → kavşak
        // Standart 20-30mm çizgiler 8 sensörün (~68mm) yarısını kapatır.
        data.state = LINE_STATE_CROSS;
        data.last_seen_ms = now;
    } else {
        // Normal çizgi takip
        data.state = LINE_STATE_ON_LINE;
        data.last_seen_ms = now;
    }

    // 4) PID kontrolü
    float error = 0.0f - data.position; // Hedef: merkez (0.0)

    pid_integral += error * dt;
    // Anti-windup
    if (pid_integral > 50.0f) pid_integral = 50.0f;
    if (pid_integral < -50.0f) pid_integral = -50.0f;

    float derivative = (dt > 0.001f) ? (error - pid_prev_error) / dt : 0.0f;
    pid_prev_error = error;

    float output = LINE_KP * error + LINE_KI * pid_integral + LINE_KD * derivative;

    // Çıktıyı sınırla
    if (output > (float)LINE_PID_MAX) output = (float)LINE_PID_MAX;
    if (output < -(float)LINE_PID_MAX) output = -(float)LINE_PID_MAX;

    data.steer_output = output;
}

// --- Getters ---

LineData* line_get_data(void)
{
    return &data;
}

float line_get_steer(void)
{
    return data.steer_output;
}

bool line_is_on_line(void)
{
    return data.state == LINE_STATE_ON_LINE;
}

bool line_is_cross(void)
{
    return data.state == LINE_STATE_CROSS;
}

const char* line_state_name(LineState s)
{
    switch (s) {
        case LINE_STATE_ON_LINE:    return "ON_LINE";
        case LINE_STATE_LOST_LEFT:  return "LOST_LEFT";
        case LINE_STATE_LOST_RIGHT: return "LOST_RIGHT";
        case LINE_STATE_LOST_ALL:   return "LOST_ALL";
        case LINE_STATE_CROSS:      return "CROSS";
        default:                    return "UNKNOWN";
    }
}
