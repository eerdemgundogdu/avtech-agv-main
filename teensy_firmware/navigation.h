#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
//  Otonom Navigasyon FSM — Yarışma Hareket-Kabiliyet Videosu Senaryosu
//  Durumlar: Başlat → Çizgi Takip → Engelde Dur → Dönüş → Yük Al → Bırak
// ============================================================================

typedef enum {
    NAV_IDLE,               // Bekleme (komut bekliyor)
    NAV_FORWARD,            // Duz ileri surus
    NAV_LINE_FOLLOW,        // Cizgi takip modunda ilerleme
    NAV_OBSTACLE_STOP,      // Engel tespit — durma
    NAV_TURN_LEFT,          // Sola 90° donus
    NAV_TURN_RIGHT,         // Saga 90° donus
    NAV_APPROACH_PICKUP,    // Yuk alma noktasina yaklasma
    NAV_PICKUP,             // Yuku alma (gripper kapat)
    NAV_APPROACH_DROP,      // Yuk birakma noktasina yaklasma
    NAV_DROP,               // Yuku birakma (gripper ac)
    NAV_RETURN,             // Baslangica donus
    NAV_SLOW_FORWARD,       // Yavas ileri (Jetson QR yaklasma)
    NAV_DONE,               // Gorev tamamlandi
    NAV_FAULT,              // Hata durumu
    NAV_MANUAL              // Manuel kontrol (Jetson'dan gelen komutlar)
} NavState;

typedef enum {
    NAV_FAULT_NONE,
    NAV_FAULT_ESTOP,        // Acil durdurma
    NAV_FAULT_MOTOR_LOST,   // Motor iletişim kaybı
    NAV_FAULT_LINE_LOST,    // Çizgi kayıp (timeout)
    NAV_FAULT_TIMEOUT,      // Görev zaman aşımı
    NAV_FAULT_IMU           // IMU hatası
} NavFault;

// Manuel surus yon tutma durumu
typedef enum {
    HOLD_OFF,               // Direksiyon veriliyor / duruyor / kapali
    HOLD_SETTLING,          // Direksiyon birakildi, donusun sonmesi bekleniyor
    HOLD_ACTIVE             // Yaw kilitli, duzeltme uygulaniyor
} HoldState;

// Motor çıktıları
typedef struct {
    int16_t speed;          // İleri/Geri Hız
    int16_t steer;          // Sağ/Sol Dönüş
} MotorOutput;

// --- API ---
void         nav_init(void);
void         nav_update(float yaw, float dt);
void         nav_imu_feed(float gyro_z, bool ok);   // nav_update'ten once her dongu (yon tutma icin)
void         nav_start_mission(void);
void         nav_stop(void);
void         nav_reset(void);
void         nav_set_manual(int16_t speed, int16_t steer);

NavState     nav_get_state(void);
NavFault     nav_get_fault(void);
MotorOutput  nav_get_output(void);
bool         nav_is_active(void);
bool         nav_is_done(void);

const char*  nav_state_name(NavState s);
const char*  nav_fault_name(NavFault f);

// Jetson komutları
void         nav_cmd_start(void);
void         nav_cmd_stop(void);
void         nav_cmd_turn(float angle_deg);
void         nav_cmd_forward(int duration_ms);
void         nav_cmd_manual(int16_t speed, int16_t steer);
void         nav_set_fused_line(float offset, const char* source);
void         nav_cmd_forklift(bool up);          // Forklift tam strok: up = en ust, down = en alt
void         nav_cmd_slow_forward(int16_t speed); // Jetson MissionControl: yavas ileri

// Yon tutma (manuel duz gidis)
void         nav_cmd_hold_enable(bool on);
bool         nav_hold_enabled(void);
HoldState    nav_hold_state(void);
float        nav_hold_target(void);

// Forklift (DM556 step) — konum takipli. Hareket sirasinda araç durur (PICKUP/DROP durumu).
bool         nav_cmd_fork_goto(int32_t target_steps);          // Mutlak konum (yazilim limitli)
bool         nav_cmd_fork_jog(int32_t steps, bool no_limit);   // Goreli (+ yukari, - asagi)
void         nav_cmd_fork_stop(void);                          // Aninda durdur (konum korunur)
void         nav_cmd_fork_zero(void);                          // Mevcut konum = 0 (alt) kalibrasyonu
void         nav_cmd_fork_hold(int8_t dir);                    // Basili tut: +1 yukari, -1 asagi (keepalive)
bool         nav_fork_hold_active(void);
int32_t      nav_fork_position(void);
bool         nav_fork_moving(void);
const char*  nav_fork_state_name(void);   // DOWN / UP / MID / MOVING_UP / MOVING_DOWN

// Guvenlik
bool         nav_estop_pressed(void);
void         nav_link_lost(void);         // Jetson kalp atisi koptu → hareketi durdur

#endif // NAVIGATION_H
