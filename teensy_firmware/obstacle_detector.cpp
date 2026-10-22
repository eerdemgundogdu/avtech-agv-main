#include "obstacle_detector.h"

// ============================================================================
//  Ultrasonik Engel Algılama — HC-SR04 (Non-blocking Interrupt-Driven)
// ============================================================================

static const int trig_pins[ULTRASONIC_COUNT] = {US_FRONT_TRIG, US_LEFT_TRIG, US_RIGHT_TRIG};
static const int echo_pins[ULTRASONIC_COUNT] = {US_FRONT_ECHO, US_LEFT_ECHO, US_RIGHT_ECHO};

static ObstacleData obs_data;
static uint8_t      current_sensor = 0;  // Round-robin sensör indeksi
static uint32_t     last_read_ms   = 0;

// Interrupt (ISR) Değişkenleri
volatile uint32_t echo_start_us[ULTRASONIC_COUNT] = {0};
volatile uint32_t echo_duration_us[ULTRASONIC_COUNT] = {0};
volatile bool     echo_ready[ULTRASONIC_COUNT] = {false};

// --- ISRs (Interrupt Service Routines) ---

void isr_front() {
    if (digitalReadFast(US_FRONT_ECHO) == HIGH) {
        echo_start_us[0] = micros();
    } else {
        echo_duration_us[0] = micros() - echo_start_us[0];
        echo_ready[0] = true;
    }
}

void isr_left() {
    if (digitalReadFast(US_LEFT_ECHO) == HIGH) {
        echo_start_us[1] = micros();
    } else {
        echo_duration_us[1] = micros() - echo_start_us[1];
        echo_ready[1] = true;
    }
}

void isr_right() {
    if (digitalReadFast(US_RIGHT_ECHO) == HIGH) {
        echo_start_us[2] = micros();
    } else {
        echo_duration_us[2] = micros() - echo_start_us[2];
        echo_ready[2] = true;
    }
}

// --- Public API ---

void obstacle_init(void)
{
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        pinMode(trig_pins[i], OUTPUT);
        pinMode(echo_pins[i], INPUT);
        digitalWrite(trig_pins[i], LOW);
    }

    // Pin kesmelerini (interrupt) ayarla
    attachInterrupt(digitalPinToInterrupt(US_FRONT_ECHO), isr_front, CHANGE);
    attachInterrupt(digitalPinToInterrupt(US_LEFT_ECHO), isr_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(US_RIGHT_ECHO), isr_right, CHANGE);

    memset(&obs_data, 0, sizeof(obs_data));
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        obs_data.distance_cm[i] = US_MAX_RANGE_CM;
    }
    obs_data.level = OBSTACLE_NONE;
    current_sensor = 0;
    last_read_ms = millis();

    Serial.println("[OBS] 3x HC-SR04 interrupt-driven ultrasonic sensors initialized");
}

void obstacle_update(void)
{
    uint32_t now = millis();

    // Round-robin: Her 20ms'de bir sensör tetikle. (Önceki okumayı işle)
    // 3 sensör x 20ms = Toplamda tüm sensörler 60ms'de taranır. (Eskiden 180ms idi)
    if ((now - last_read_ms) >= 20) {
        last_read_ms = now;

        // 1. Önceki tetiklenen sensörün sonucunu al
        if (echo_ready[current_sensor]) {
            // Ses hızı: 343 m/s → ~29.15 µs/cm → gidiş-dönüş / 2
            float distance = (float)echo_duration_us[current_sensor] / 58.3f;
            if (distance > US_MAX_RANGE_CM) distance = US_MAX_RANGE_CM;
            obs_data.distance_cm[current_sensor] = distance;
            echo_ready[current_sensor] = false;
        } else {
            // Timeout veya sinyal dönmedi (20ms > 12ms donanım limiti)
            obs_data.distance_cm[current_sensor] = US_MAX_RANGE_CM;
        }

        // 2. Sonraki sensöre geç
        current_sensor = (current_sensor + 1) % ULTRASONIC_COUNT;

        // 3. Yeni sensörü tetikle (Sadece 10 mikrosaniye bekler, sistemi bloke etmez)
        echo_ready[current_sensor] = false; // Temizle
        digitalWriteFast(trig_pins[current_sensor], HIGH);
        delayMicroseconds(10);
        digitalWriteFast(trig_pins[current_sensor], LOW);
    }

    // Engel durumlarını güncelle
    float front = obs_data.distance_cm[0];
    float left  = obs_data.distance_cm[1];
    float right = obs_data.distance_cm[2];

    obs_data.front_blocked = (front < US_STOP_DISTANCE_CM);
    obs_data.left_blocked  = (left  < US_STOP_DISTANCE_CM);
    obs_data.right_blocked = (right < US_STOP_DISTANCE_CM);

    // Genel engel seviyesi (ön sensöre göre)
    if (front < US_STOP_DISTANCE_CM) {
        obs_data.level = OBSTACLE_CLOSE;
    } else if (front < US_SLOW_DISTANCE_CM) {
        obs_data.level = OBSTACLE_FAR;
    } else {
        obs_data.level = OBSTACLE_NONE;
    }
}

ObstacleData* obstacle_get_data(void)
{
    return &obs_data;
}

bool obstacle_is_blocked(void)
{
    return obs_data.front_blocked;
}

ObstacleLevel obstacle_get_level(void)
{
    return obs_data.level;
}
