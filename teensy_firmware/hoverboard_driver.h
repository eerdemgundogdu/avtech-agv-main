#ifndef HOVERBOARD_DRIVER_H
#define HOVERBOARD_DRIVER_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
//  Hoverboard FOC Firmware — USART Protocol (VARIANT_USART + SPD_MODE)
//  Referans: https://github.com/EFeru/hoverboard-firmware-hack-FOC
//
//  SPD_MODE: Motor firmware dahili PID ile hedef RPM'e kilitlenir.
//  Input aralığı: -1000 ↔ +1000 (doğrudan hedef hız oranı)
//
//  Kritik NOT: FOC firmware config.h'da şu ayarlar yapılmış olmalıdır:
//    #define VARIANT_USART
//    #define CTRL_TYP_SEL    FOC_CTRL
//    #define CTRL_MOD_REQ    SPD_MODE
// ============================================================================

// ── Komut paketi (Teensy → Hoverboard) ──────────────────────────────────────
// Start frame: 0xABCD (little-endian: first byte 0xCD, second 0xAB)
// Payload: steer (int16), speed (int16)
// Checksum: XOR of all preceding 16-bit words
typedef struct __attribute__((packed)) {
    uint16_t start;       // START_FRAME = 0xABCD
    int16_t  steer;       // Dönüş komutu (-1000 ↔ +1000)
    int16_t  speed;       // Hız komutu   (-1000 ↔ +1000)
    uint16_t checksum;    // XOR checksum (word-level)
} HoverCommand;

// ── Feedback paketi (Hoverboard → Teensy) ───────────────────────────────────
typedef struct __attribute__((packed)) {
    uint16_t start;       // START_FRAME = 0xABCD
    int16_t  cmd1;        // Motor 1 komutu (echo)
    int16_t  cmd2;        // Motor 2 komutu (echo)
    int16_t  speedR_meas; // Sağ motor hız ölçümü (RPM * dahili çarpan)
    int16_t  speedL_meas; // Sol motor hız ölçümü (RPM * dahili çarpan)
    int16_t  batVoltage;  // Batarya gerilimi (x100, ör: 2400 = 24.00V)
    int16_t  boardTemp;   // Kart sıcaklığı (x10, ör: 385 = 38.5°C)
    uint16_t cmdLed;      // LED kontrol (kullanılmıyor)
    uint16_t checksum;    // XOR checksum (word-level)
} HoverFeedback;

// ── Motor durumu (genişletilmiş) ────────────────────────────────────────────
typedef struct {
    bool     connected;          // Feedback alınıyor mu
    int16_t  speed_meas;         // Ölçülen hız — aktif motor (RPM birim)
    int16_t  speedR_raw;         // Sağ motor ham hız (her iki motoru da görmek için)
    int16_t  speedL_raw;         // Sol motor ham hız
    int16_t  cmd_echo;           // Echo edilen komut
    int16_t  bat_voltage;        // Batarya (x100 V)
    int16_t  board_temp;         // Sıcaklık (x10 °C)
    uint32_t last_recv_ms;       // Son feedback zamanı
    uint32_t recv_count;         // Toplam alınan paket sayısı
    uint32_t checksum_errors;    // Checksum hata sayacı
} MotorStatus;

// ── Odometri verisi ─────────────────────────────────────────────────────────
// Hoverboard Hall sensor tick'lerinden kümülatif yol hesaplama
typedef struct {
    float    distance_m;         // Toplam kat edilen yol (metre)
    float    velocity_mps;       // Anlık hız (m/s)
    float    velocity_filtered;  // Düşük geçiş filtreli hız
    uint32_t last_update_ms;
} OdometryData;

// ── Ramping (yumuşak hız geçişi) ────────────────────────────────────────────
typedef struct {
    int16_t  current;            // Mevcut (ramplanmış) hız
    int16_t  target;             // Hedef hız
    uint32_t last_ramp_ms;       // Son ramp güncelleme zamanı
} RampState;

// ── Public API ──────────────────────────────────────────────────────────────

// Başlatma ve kapatma
void    hoverboard_init(void);

// Ramping'li hız komutu gönder (ana fonksiyon)
// speed / steer: -MOTOR_SPEED_MAX ↔ +MOTOR_SPEED_MAX
void    hoverboard_send(int16_t speed, int16_t steer);

// Ramping bypass — acil/doğrudan hız komutu (ramping yok!)
void    hoverboard_send_direct(int16_t speed, int16_t steer);

// Acil durdurma (ramping bypass, anında 0)
void    hoverboard_stop(void);

// Yumuşak durdurma (ramping ile 0'a iner)
void    hoverboard_soft_stop(void);

// Feedback oku (loop'ta her döngü çağırılmalı)
void    hoverboard_receive(void);

// Ramping güncelle (loop'ta her döngü çağırılmalı)
void    hoverboard_update_ramp(void);

// Odometriyi güncelle (loop'ta her döngü çağırılmalı)
void    hoverboard_update_odometry(float dt);

// ── Durum Sorguları ─────────────────────────────────────────────────────────

bool    hoverboard_left_connected(void);
bool    hoverboard_right_connected(void);
bool    hoverboard_any_connected(void);

const MotorStatus*  hoverboard_get_left(void);
const MotorStatus*  hoverboard_get_right(void);
const OdometryData* hoverboard_get_odometry_left(void);
const OdometryData* hoverboard_get_odometry_right(void);

float   hoverboard_get_battery_voltage(void);
float   hoverboard_get_avg_speed(void);        // Ortalama RPM
float   hoverboard_get_avg_velocity_mps(void); // Ortalama m/s
float   hoverboard_get_total_distance(void);   // Toplam yol (metre)

// Hız doğrulama — komut edilen ile ölçülen arasındaki fark
float   hoverboard_get_speed_error_left(void);
float   hoverboard_get_speed_error_right(void);
bool    hoverboard_speed_within_tolerance(float tolerance_pct);

#endif // HOVERBOARD_DRIVER_H
