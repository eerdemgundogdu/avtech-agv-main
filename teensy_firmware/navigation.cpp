#include "navigation.h"
#include "line_follower.h"
#include "obstacle_detector.h"
#include "imu_handler.h"
#include "hoverboard_driver.h"

// ============================================================================
//  Otonom Navigasyon FSM — Yarışma Senaryosu
//  Yük mekanizması: DM556 Step Motor Driver (STEP/DIR/ENA sinyalleri)
//
//  Forklift: adimlar IntervalTimer kesmesinde uretilir ve konum (adim) sayilir.
//  0 = en alt, FORK_TOP_STEPS = en ust. Yazilim limitleri tam strokun disina cikmayi engeller.
// ============================================================================

static NavState   state = NAV_IDLE;
static NavFault   fault = NAV_FAULT_NONE;
static MotorOutput output = {0, 0};

static uint32_t state_start_ms   = 0;
static uint32_t mission_start_ms = 0;
static float    target_yaw       = 0.0f;
static int      mission_step     = 0;
static int16_t  manual_speed     = 0;
static int16_t  manual_steer     = 0;
static uint32_t manual_last_cmd_ms = 0;   // MANUAL failsafe: son manuel komut zamanı
static int16_t  slow_forward_speed = 80;  // Jetson yavas ileri hiz

// İleri komut parametreleri
static int32_t cmd_forward_duration_ms = 0;

// Cross sayacı — kavşaklarda dönüş kararı
static int cross_count = 0;

// Manuel surus yon tutma (IMU)
static bool      hold_enabled   = HOLD_ENABLED_DEFAULT;
static HoldState hold_state     = HOLD_OFF;
static float     hold_target    = 0.0f;
static float     hold_integral  = 0.0f;      // steer birimi
static float     hold_corr      = 0.0f;      // Son hesaplanan duzeltme (steer birimi)
static uint32_t  hold_since_ms  = 0;
static uint32_t  hold_last_ms   = 0;
static float     imu_gyro_z     = 0.0f;
static bool      imu_valid      = false;

static float    fused_line_offset_cm = 0.0f;
static char     fused_line_source[16] = "";
static uint32_t fused_line_last_ms = 0;

// --- Forklift (DM556) ---

static IntervalTimer    fork_timer;
static volatile int32_t fork_pos    = 0;      // Mevcut konum (adim)
static volatile int32_t fork_target = 0;      // Hedef konum (adim)
static volatile int8_t  fork_dir    = 0;      // +1 yukari, -1 asagi
static volatile bool    fork_moving = false;  // ISR adim uretiyor mu
static bool             fork_enabled = false; // DM556 EN aktif mi
static bool             fork_initialized = false;
static NavState         fork_return_state = NAV_IDLE;  // Forklift bitince donulecek durum
static bool             fork_slow = false;             // Basili tut modu: yavas adim hizi
static bool             fork_hold_active = false;      // GUI basili tut hareketi suruyor
static int8_t           fork_hold_dir = 0;
static uint32_t         fork_hold_ms = 0;              // Son basili tut keepalive zamani

// STEPPER_PULSE_DELAY_US periyodunda (500us = 2 kHz) cagrilir. Hedefe ulasinca kendiliginden durur.
static void fork_isr(void)
{
    if (!fork_moving) return;
    if (fork_pos == fork_target) {
        fork_moving = false;
        return;
    }
    digitalWriteFast(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_PULSE_US);
    digitalWriteFast(STEPPER_STEP_PIN, LOW);
    fork_pos += fork_dir;
}

static void stepper_enable(void)
{
    if (!fork_enabled) {
        digitalWriteFast(STEPPER_EN_PIN, LOW);  // LOW = aktif
        delayMicroseconds(100);                 // Enable stabilizasyon
        fork_enabled = true;
    }
}

static void stepper_disable(void)
{
    digitalWriteFast(STEPPER_EN_PIN, HIGH);     // HIGH = devre dışı
    fork_enabled = false;
}

static void stepper_init(void)
{
    if (fork_initialized) return;
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);

    digitalWriteFast(STEPPER_STEP_PIN, LOW);
    digitalWriteFast(STEPPER_DIR_PIN, LOW);
    stepper_disable();

    fork_timer.begin(fork_isr, STEPPER_PULSE_DELAY_US);
    fork_initialized = true;
    Serial.println("[NAV] DM556 stepper driver initialized (IntervalTimer)");
}

// Adim hizi: normal (tam strok / goto / jog) veya yavas (basili tut)
static void fork_set_slow(bool slow)
{
    if (slow == fork_slow) return;
    fork_slow = slow;
    fork_timer.update(slow ? FORK_SLOW_PULSE_DELAY_US : STEPPER_PULSE_DELAY_US);
}

// Hareketi aninda keser. Konum ISR'da sayildigi icin dogru kalir.
static void fork_halt(void)
{
    fork_hold_active = false;
    fork_moving = false;
    fork_target = fork_pos;
    if (!FORK_HOLD_TORQUE) stepper_disable();
}

// Hedefe hareketi baslatir (non-blocking). Yon degisimi ISR beklerken yapilir.
static void fork_start_move(int32_t target)
{
    if (!fork_initialized) stepper_init();

    fork_moving = false;                 // ISR'i beklet (yon pini guvenle degissin)
    int32_t pos = fork_pos;
    if (target == pos) {
        fork_target = pos;
        return;
    }
    bool up = target > pos;
    digitalWriteFast(STEPPER_DIR_PIN, up ? FORK_DIR_UP : FORK_DIR_DOWN);
    stepper_enable();
    delayMicroseconds(50);               // DIR stabilizasyon (DM556: >5us)
    fork_dir    = up ? 1 : -1;
    fork_target = target;
    fork_moving = true;
}

// Ana donguden: hareket bitince surucuyu birak (eski davranis, FORK_HOLD_TORQUE=0)
static void fork_service(void)
{
    if (!fork_moving && fork_enabled && !FORK_HOLD_TORQUE) {
        stepper_disable();
    }
}

static int32_t fork_clamp(int32_t t)
{
    if (t < 0) return 0;
    if (t > FORK_TOP_STEPS) return FORK_TOP_STEPS;
    return t;
}

// Otonom FSM'in eski API'si: gripper_close = yukari (yuku al), gripper_open = asagi (birak)
static void gripper_open(void)
{
    Serial.println("[NAV] Gripper OPEN / forklift DOWN (stepper)");
    fork_start_move(0);
}

static void gripper_close(void)
{
    Serial.println("[NAV] Gripper CLOSE / forklift UP (stepper)");
    fork_start_move(FORK_TOP_STEPS);
}

// --- FSM yardimcilari ---

static uint32_t elapsed_ms(void)
{
    return millis() - state_start_ms;
}

static void enter_state(NavState next)
{
    state = next;
    state_start_ms = millis();
    output = {0, 0};
    imu_heading_reset_pid();

    Serial.print("[NAV] -> ");
    Serial.println(nav_state_name(next));
}

static void enter_fault(NavFault f)
{
    fault = f;
    enter_state(NAV_FAULT);
    Serial.print("[NAV] FAULT: ");
    Serial.println(nav_fault_name(f));
}

static void drive(int16_t speed, int16_t steer)
{
    output.speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    output.steer = constrain(steer, -MOTOR_STEER_MAX, MOTOR_STEER_MAX);
}

// Manevra (ileri/donus) bitince: manuel modda bekle, otonomda cizgi takibe gec
static NavState after_maneuver_state(void)
{
    return MANUAL_ONLY_MODE ? NAV_IDLE : NAV_LINE_FOLLOW;
}

static void reject(const char *cmd, const char *reason)
{
    JETSON_SERIAL.print("{\"resp\":\"rejected\",\"cmd\":\"");
    JETSON_SERIAL.print(cmd);
    JETSON_SERIAL.print("\",\"reason\":\"");
    JETSON_SERIAL.print(reason);
    JETSON_SERIAL.println("\"}");
}

static void report_fork_done(void)
{
    JETSON_SERIAL.print("{\"status\":\"forklift_done\",\"pos\":");
    JETSON_SERIAL.print((long)fork_pos);
    JETSON_SERIAL.println("}");
}

// --- Public API ---

void nav_init(void)
{
    state = NAV_IDLE;
    fault = NAV_FAULT_NONE;
    output = {0, 0};
    mission_step = 0;
    cross_count = 0;
    manual_speed = 0;
    manual_steer = 0;

    pinMode(ESTOP_PIN, INPUT_PULLUP);
    stepper_init();

#if FORK_BOOT_HOME_DOWN
    // Eski davranis: acilista tam strok asagi (alt dayanaga yaslanir) → konum 0
    if (!nav_estop_pressed()) {
        fork_pos = FORK_TOP_STEPS;
        gripper_open();
        Serial.println("[NAV] Forklift acilis: alta suruluyor (FORK_BOOT_HOME_DOWN)");
    }
#else
    fork_pos = 0;
    fork_target = 0;
#endif

    Serial.println("[NAV] Navigation FSM initialized");
}

void nav_start_mission(void)
{
#if MANUAL_ONLY_MODE
    reject("start", "MANUAL_ONLY_MODE");
    Serial.println("[NAV] Otonom gorev reddedildi: MANUAL_ONLY_MODE=1 (cizgi/engel sensorleri kapali)");
    return;
#endif
    if (state == NAV_IDLE || state == NAV_DONE || state == NAV_FAULT) {
        if (nav_estop_pressed()) { reject("start", "ESTOP"); return; }
        fault = NAV_FAULT_NONE;
        mission_start_ms = millis();
        mission_step = 0;
        cross_count = 0;
        enter_state(NAV_FORWARD);
        Serial.println("[NAV] ===== MISSION STARTED =====");
    }
}

void nav_stop(void)
{
    output = {0, 0};
    mission_start_ms = 0;
    manual_speed = 0;
    manual_steer = 0;
    hoverboard_stop();
    fork_halt();
    mission_step = 0;
    enter_state(NAV_IDLE);
    Serial.println("[NAV] Mission stopped by command");
}

void nav_reset(void)
{
    fault = NAV_FAULT_NONE;
    output = {0, 0};
    mission_step = 0;
    cross_count = 0;
    mission_start_ms = 0;
    manual_speed = 0;
    manual_steer = 0;
    hoverboard_stop();
    fork_halt();
    enter_state(NAV_IDLE);
}

void nav_set_manual(int16_t speed, int16_t steer)
{
    manual_speed = speed;
    manual_steer = steer;
    manual_last_cmd_ms = millis();
}

bool nav_estop_pressed(void)
{
#if ESTOP_ENABLED
    return digitalRead(ESTOP_PIN) == LOW;
#else
    return false;   // Buton bagli degil (config.h ESTOP_ENABLED)
#endif
}

void nav_imu_feed(float gyro_z, bool ok)
{
    imu_gyro_z = gyro_z;
    imu_valid  = ok;
}

static void hold_off(void)
{
    hold_state    = HOLD_OFF;
    hold_integral = 0.0f;
    hold_corr     = 0.0f;
}

// Manuel surus: sadece ileri/geri komutu varken (steer = 0) IMU ile yaw'i kilitler.
// Direksiyon verilince, durunca veya IMU yoksa serbest. 50 Hz'de hesaplanir (dt sabit, turev = gyro).
static int16_t manual_hold_steer(int16_t speed, int16_t steer, float yaw)
{
    uint32_t now = millis();
    if (!hold_enabled || !imu_valid || steer != 0 || abs(speed) < HOLD_MIN_SPEED) {
        hold_off();
        return 0;
    }
    if (hold_state == HOLD_OFF) {
        hold_state    = HOLD_SETTLING;
        hold_since_ms = now;
        hold_integral = 0.0f;
        hold_corr     = 0.0f;
    }
    if (hold_state == HOLD_SETTLING) {
        // Onceki donusun atalet ile devam eden kismini hedefe katma
        if (fabsf(imu_gyro_z) < HOLD_SETTLE_GYRO_DPS || (now - hold_since_ms) >= HOLD_SETTLE_MS) {
            hold_state   = HOLD_ACTIVE;
            hold_target  = yaw;
            hold_last_ms = now;
        }
        return 0;
    }

    if ((now - hold_last_ms) >= HOLD_PERIOD_MS) {
        float dt = (float)(now - hold_last_ms) / 1000.0f;
        if (dt > 0.1f) dt = 0.1f;
        hold_last_ms = now;

        float err = imu_angle_diff(hold_target, yaw);   // + = hedef solda
        if (fabsf(err) > HOLD_DIVERGE_DEG) {
            // Arac elle itildi / tekerlek kaydi / IMU ters takili: zorla geri cevirme
            JETSON_SERIAL.print("{\"status\":\"hold_diverge\",\"err\":");
            JETSON_SERIAL.print(err, 1);
            JETSON_SERIAL.println("}");
            hold_target   = yaw;         // Mevcut yonu yeni hedef kabul et
            hold_integral = 0.0f;
            hold_corr     = 0.0f;
            return 0;
        }
        // Yaw sola donuste artar; pozitif steer araci saga dondurur → isaret ters.
        hold_integral += HOLD_KI * err * dt;
        hold_integral = constrain(hold_integral, -HOLD_I_MAX, HOLD_I_MAX);
        float u = HOLD_KP * err + hold_integral - HOLD_KD * imu_gyro_z;
        hold_corr = constrain(-u, -(float)HOLD_MAX_STEER, (float)HOLD_MAX_STEER);
    }
    return (int16_t)hold_corr;
}

void nav_cmd_hold_enable(bool on)
{
    hold_enabled = on;
    hold_off();
    Serial.println(on ? "[NAV] Yon tutma ACIK" : "[NAV] Yon tutma KAPALI");
}

bool      nav_hold_enabled(void) { return hold_enabled; }
HoldState nav_hold_state(void)   { return hold_state; }
float     nav_hold_target(void)  { return hold_target; }

void nav_link_lost(void)
{
    if (nav_is_active() || fork_moving) {
        Serial.println("[NAV] Jetson kalp atisi kesildi — hareket durduruldu");
        nav_stop();
    }
}

void nav_update(float yaw, float dt)
{
    fork_service();

    // Basili tut forklift: GUI keepalive kesildi (tus birakildi, ag koptu) → dur
    if (fork_hold_active && (millis() - fork_hold_ms) > FORK_HOLD_TIMEOUT_MS) {
        fork_halt();
        Serial.println("[NAV] Forklift basili tut sona erdi");
    }
    if (state != NAV_MANUAL && hold_state != HOLD_OFF) hold_off();

    // Acil durdurma — surus VE forklift aninda durur
    if (nav_estop_pressed()) {
        output = {0, 0};
        if (fork_moving) fork_halt();
        if (state != NAV_FAULT) {
            hoverboard_stop();
            enter_fault(NAV_FAULT_ESTOP);
        }
        return;
    }

    // Görev zaman aşımı (MANUAL sürüş görev süresine dahil değil)
    if (nav_is_active() && state != NAV_MANUAL && mission_start_ms > 0) {
        if ((millis() - mission_start_ms) > NAV_MISSION_TIMEOUT_MS) {
            fork_halt();
            enter_fault(NAV_FAULT_TIMEOUT);
            return;
        }
    }

    LineData *line = line_get_data();

    switch (state) {
        case NAV_IDLE:
            output = {0, 0};
            break;

        case NAV_FORWARD:
            if (mission_step == 0) {
                target_yaw = yaw;
                mission_step = 1;
            }
            {
                float steer = imu_heading_hold(target_yaw, yaw, dt);
                drive(LINE_BASE_SPEED, (int16_t)steer);
            }
            {
                uint32_t fwd_dur = (cmd_forward_duration_ms > 0)
                                   ? (uint32_t)cmd_forward_duration_ms
                                   : (uint32_t)NAV_FORWARD_MS;
                // Robot çizgiyi görene kadar düz ilerler; zamanlayıcı güvenlik amaçlıdır.
                // MANUAL_ONLY_MODE'da QTR kapali (veri yok) oldugu icin sadece sure kullanilir.
                bool line_seen = !MANUAL_ONLY_MODE && line_is_on_line();
                if (line_seen || elapsed_ms() >= fwd_dur) {
                    mission_step = 0;
                    cmd_forward_duration_ms = 0;
                    enter_state(after_maneuver_state());
                }
            }
            break;

        case NAV_LINE_FOLLOW:
            if (MANUAL_ONLY_MODE) {      // Sensor yok: guvenli tarafta kal
                enter_state(NAV_IDLE);
                break;
            }
            if (line_is_cross()) {
                cross_count++;
                Serial.print("[NAV] Cross detected #");
                Serial.println(cross_count);

                if (cross_count == 1) {
                    target_yaw = yaw + 90.0f;
                    if (target_yaw >= 360.0f) target_yaw -= 360.0f;
                    enter_state(NAV_TURN_RIGHT);
                } else if (cross_count == 2) {
                    enter_state(NAV_APPROACH_PICKUP);
                } else if (cross_count == 3) {
                    target_yaw = yaw - 90.0f;
                    if (target_yaw < 0.0f) target_yaw += 360.0f;
                    enter_state(NAV_TURN_LEFT);
                } else if (cross_count == 4) {
                    enter_state(NAV_APPROACH_DROP);
                } else {
                    enter_state(NAV_DONE);
                }
                break;
            }

            if (line_is_on_line()) {
                float steer = line_get_steer();
                drive(LINE_BASE_SPEED, (int16_t)steer);
            } else if (line->state == LINE_STATE_LOST_ALL) {
                drive(LINE_BASE_SPEED / 2, 0);
                if ((millis() - line->last_seen_ms) > LINE_LOST_TIMEOUT_MS) {
                    enter_fault(NAV_FAULT_LINE_LOST);
                }
            } else {
                int16_t recovery_steer = (line->state == LINE_STATE_LOST_LEFT) ? -80 : 80;
                drive(LINE_BASE_SPEED / 3, recovery_steer);
            }
            break;

        case NAV_OBSTACLE_STOP:
            // [DEVRE DISI] Ultrasonik engel durdurma video aşaması için kapatıldı
            mission_step = 0;
            enter_state(after_maneuver_state());
            break;

        case NAV_TURN_RIGHT:
        case NAV_TURN_LEFT:
            {
                float heading_error = imu_angle_diff(target_yaw, yaw);
                float steer = imu_heading_hold(target_yaw, yaw, dt);
                drive(0, (int16_t)steer);

                if (fabsf(heading_error) < HEADING_TOLERANCE_DEG &&
                    elapsed_ms() >= HEADING_SETTLE_MS) {
                    Serial.println(state == NAV_TURN_RIGHT ? "[NAV] Right turn completed"
                                                           : "[NAV] Left turn completed");
                    enter_state(after_maneuver_state());
                } else if (elapsed_ms() >= (uint32_t)NAV_TURN_TIMEOUT_MS) {
                    Serial.println("[NAV] Turn timeout, continuing");
                    enter_state(after_maneuver_state());
                }
            }
            break;

        case NAV_APPROACH_PICKUP:
        case NAV_APPROACH_DROP:
            {
                float steer = 0.0f;
                // Jetson'dan gelen asenkron kamera/fusion verisi (500ms timeout)
                if (millis() - fused_line_last_ms < 500 &&
                    (strcmp(fused_line_source, "camera") == 0 || strcmp(fused_line_source, "both") == 0)) {
                    float Kp_camera = 15.0f;
                    steer = fused_line_offset_cm * Kp_camera;
                } else if (line_is_on_line()) {
                    steer = line_get_steer();   // Kamera verisi yoksa QTR-8A (fallback)
                }

                drive(LINE_BASE_SPEED / 3, (int16_t)steer);

                uint32_t approach_time = (state == NAV_APPROACH_PICKUP) ? (uint32_t)NAV_PICKUP_APPROACH_MS
                                                                        : (uint32_t)NAV_DROP_APPROACH_MS;
                if (elapsed_ms() >= approach_time) {
                    bool pickup = (state == NAV_APPROACH_PICKUP);
                    output = {0, 0};
                    mission_step = 0;   // PICKUP/DROP hareketi kendisi baslatsin
                    fork_return_state = MANUAL_ONLY_MODE ? NAV_IDLE
                                      : (pickup ? NAV_LINE_FOLLOW : NAV_DONE);
                    enter_state(pickup ? NAV_PICKUP : NAV_DROP);
                }
            }
            break;

        case NAV_PICKUP:
        case NAV_DROP:
            // Forklift hareket ederken araç durur
            output = {0, 0};
            if (mission_step == 0) {
                if (state == NAV_PICKUP) gripper_close();
                else                     gripper_open();
                mission_step = 1;
            }
            if (mission_step == 1 && !fork_moving) {
                Serial.println(state == NAV_PICKUP ? "[NAV] Load picked up / forklift up done"
                                                   : "[NAV] Load dropped / forklift down done");
                report_fork_done();
                mission_step = 0;
                NavState next = fork_return_state;
                fork_return_state = NAV_IDLE;
                enter_state(next);
            }
            break;

        case NAV_RETURN:
            if (line_is_on_line()) {
                drive(LINE_BASE_SPEED, (int16_t)line_get_steer());
            }
            if (elapsed_ms() >= 10000) {
                enter_state(NAV_DONE);
            }
            break;

        case NAV_DONE:
            output = {0, 0};
            break;

        case NAV_FAULT:
            output = {0, 0};
            break;

        case NAV_MANUAL:
            // FAILSAFE: Jetson/USB/kumanda zinciri koparsa son komutla gitmeye devam etme
            // Rampa ile durur (ani fren yok); acil durdurma icin STOP komutu aninda keser.
            if ((millis() - manual_last_cmd_ms) > MANUAL_CMD_TIMEOUT_MS) {
                manual_speed = 0;
                manual_steer = 0;
                hold_off();
                Serial.println("[NAV] MANUAL komut zaman asimi — motorlar durduruluyor");
                enter_state(NAV_IDLE);
                break;
            }
            drive(manual_speed, manual_steer + manual_hold_steer(manual_speed, manual_steer, yaw));
            break;

        case NAV_SLOW_FORWARD:
            {
                float steer = 0.0f;
                if (millis() - fused_line_last_ms < 500 &&
                    (strcmp(fused_line_source, "camera") == 0 || strcmp(fused_line_source, "both") == 0)) {
                    float Kp_camera = 15.0f;
                    steer = fused_line_offset_cm * Kp_camera;
                } else if (!MANUAL_ONLY_MODE && line_is_on_line()) {
                    steer = line_get_steer();
                }
                drive(slow_forward_speed, (int16_t)steer);
            }
            break;

        default:
            enter_fault(NAV_FAULT_TIMEOUT);
            break;
    }

    // Buzzer: FAULT'ta kesikli, DONE'da ilk 1 sn surekli, diger durumlarda sessiz
    if (state == NAV_FAULT) {
        digitalWrite(BUZZER_PIN, ((elapsed_ms() / 200) % 2 == 0) ? HIGH : LOW);
    } else if (state == NAV_DONE) {
        digitalWrite(BUZZER_PIN, elapsed_ms() < 1000 ? HIGH : LOW);
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

// --- Jetson komutlari ---

void nav_cmd_start(void)
{
    nav_start_mission();
}

void nav_cmd_stop(void)
{
    nav_stop();
}

void nav_cmd_turn(float angle_deg)
{
#if MANUAL_ONLY_MODE
    reject("turn", "MANUAL_ONLY_MODE");
    return;
#endif
    if (state == NAV_FAULT) { reject("turn", "FAULT"); return; }
    if (state == NAV_PICKUP || state == NAV_DROP) { reject("turn", "FORKLIFT_BUSY"); return; }

    IMU_Data_AGV imu_data_temp;
    imu_agv_read(&imu_data_temp);
    target_yaw = imu_data_temp.yaw + angle_deg;
    while (target_yaw >= 360.0f) target_yaw -= 360.0f;
    while (target_yaw < 0.0f) target_yaw += 360.0f;

    enter_state(angle_deg > 0 ? NAV_TURN_RIGHT : NAV_TURN_LEFT);
}

void nav_cmd_forward(int duration_ms)
{
#if MANUAL_ONLY_MODE
    reject("forward", "MANUAL_ONLY_MODE");
    return;
#endif
    if (state == NAV_FAULT) { reject("forward", "FAULT"); return; }
    if (state == NAV_PICKUP || state == NAV_DROP) { reject("forward", "FORKLIFT_BUSY"); return; }
    cmd_forward_duration_ms = constrain(duration_ms, 0, 30000);
    mission_step = 0;
    enter_state(NAV_FORWARD);
}

void nav_cmd_manual(int16_t speed, int16_t steer)
{
    // Sadece IDLE/MANUAL'dan kabul edilir: FAULT (E-stop) RESET ister,
    // forklift hareket ederken (PICKUP/DROP) surus komutlari yok sayilir.
    if (state == NAV_IDLE || state == NAV_MANUAL) {
        if (state != NAV_MANUAL) enter_state(NAV_MANUAL);
        nav_set_manual(speed, steer);
    }
}

void nav_set_fused_line(float offset, const char* source)
{
    fused_line_offset_cm = offset;
    strncpy(fused_line_source, source, sizeof(fused_line_source) - 1);
    fused_line_source[sizeof(fused_line_source) - 1] = '\0';
    fused_line_last_ms = millis();
}

// Komutla tetiklenen forklift hareketi: araç durur, hareket bitince IDLE'a donulur.
static bool fork_command_move(int32_t target)
{
    if (state == NAV_FAULT || nav_estop_pressed()) {
        reject("forklift", "FAULT");
        return false;
    }
    int32_t pos = fork_pos;
    if (target == pos && !fork_moving) {
        Serial.println("[NAV] Forklift zaten hedefte");
        report_fork_done();
        return true;
    }
    fork_start_move(target);
    fork_return_state = NAV_IDLE;
    mission_step = 1;                        // Hareket zaten basladi
    enter_state(target > pos ? NAV_PICKUP : NAV_DROP);
    return true;
}

void nav_cmd_forklift(bool up)
{
    Serial.println(up ? "[NAV] CMD: Forklift UP" : "[NAV] CMD: Forklift DOWN");
    fork_hold_active = false;
    fork_set_slow(false);
    fork_command_move(up ? FORK_TOP_STEPS : 0);
}

bool nav_cmd_fork_goto(int32_t target_steps)
{
    fork_hold_active = false;
    fork_set_slow(false);
    return fork_command_move(fork_clamp(target_steps));
}

void nav_cmd_fork_hold(int8_t dir)
{
    if (dir == 0) { nav_cmd_fork_stop(); return; }
    dir = dir > 0 ? 1 : -1;
    fork_hold_ms = millis();
    if (fork_hold_active && fork_hold_dir == dir) return;   // Keepalive: hareket suruyor / limitte bekliyor

    if (state == NAV_FAULT || nav_estop_pressed()) { reject("forklift", "FAULT"); return; }
    int32_t target = dir > 0 ? FORK_TOP_STEPS : 0;
    fork_set_slow(true);
    fork_hold_dir = dir;
    if (fork_pos == target && !fork_moving) {
        fork_hold_active = true;                 // Limitte: keepalive'lar sessizce yutulsun
        Serial.println("[NAV] Forklift zaten limitte");
        return;
    }
    if (fork_command_move(target)) fork_hold_active = true;
}

bool nav_fork_hold_active(void)
{
    return fork_hold_active;
}

bool nav_cmd_fork_jog(int32_t steps, bool no_limit)
{
    fork_hold_active = false;
    fork_set_slow(false);
    steps = constrain(steps, -FORK_JOG_MAX_STEPS, FORK_JOG_MAX_STEPS);
    int32_t base = fork_moving ? fork_target : fork_pos;
    int32_t target = base + steps;
    if (!no_limit) target = fork_clamp(target);
    return fork_command_move(target);
}

void nav_cmd_fork_stop(void)
{
    fork_halt();
    Serial.print("[NAV] CMD: Forklift STOP, pos=");
    Serial.println((long)fork_pos);
    // PICKUP/DROP durumu bir sonraki nav_update'te !fork_moving gorup IDLE'a doner
}

void nav_cmd_fork_zero(void)
{
    fork_halt();
    fork_pos = 0;
    fork_target = 0;
    Serial.println("[NAV] CMD: Forklift konum sifirlandi (alt = 0)");
}

int32_t nav_fork_position(void)
{
    return fork_pos;
}

bool nav_fork_moving(void)
{
    return fork_moving;
}

const char* nav_fork_state_name(void)
{
    if (fork_moving) return (fork_dir > 0) ? "MOVING_UP" : "MOVING_DOWN";
    int32_t p = fork_pos;
    if (p <= 0) return "DOWN";
    if (p >= FORK_TOP_STEPS) return "UP";
    return "MID";
}

void nav_cmd_slow_forward(int16_t speed)
{
#if MANUAL_ONLY_MODE
    reject("slow_forward", "MANUAL_ONLY_MODE");
    return;
#endif
    if (state == NAV_FAULT) { reject("slow_forward", "FAULT"); return; }
    if (state == NAV_PICKUP || state == NAV_DROP) { reject("slow_forward", "FORKLIFT_BUSY"); return; }
    slow_forward_speed = constrain(speed, 30, MOTOR_SPEED_MAX);
    Serial.print("[NAV] Jetson CMD: Slow forward speed=");
    Serial.println(slow_forward_speed);
    enter_state(NAV_SLOW_FORWARD);
}

// --- Sorgular ---

NavState nav_get_state(void)
{
    return state;
}

NavFault nav_get_fault(void)
{
    return fault;
}

MotorOutput nav_get_output(void)
{
    return output;
}

bool nav_is_active(void)
{
    return state != NAV_IDLE && state != NAV_DONE && state != NAV_FAULT;
}

bool nav_is_done(void)
{
    return state == NAV_DONE;
}

const char* nav_state_name(NavState s)
{
    switch (s) {
        case NAV_IDLE:             return "IDLE";
        case NAV_FORWARD:          return "FORWARD";
        case NAV_LINE_FOLLOW:      return "LINE_FOLLOW";
        case NAV_OBSTACLE_STOP:    return "OBSTACLE_STOP";
        case NAV_TURN_LEFT:        return "TURN_LEFT";
        case NAV_TURN_RIGHT:       return "TURN_RIGHT";
        case NAV_APPROACH_PICKUP:  return "APPROACH_PICKUP";
        case NAV_PICKUP:           return "PICKUP";
        case NAV_APPROACH_DROP:    return "APPROACH_DROP";
        case NAV_DROP:             return "DROP";
        case NAV_RETURN:           return "RETURN";
        case NAV_SLOW_FORWARD:     return "SLOW_FORWARD";
        case NAV_DONE:             return "DONE";
        case NAV_FAULT:            return "FAULT";
        case NAV_MANUAL:           return "MANUAL";
        default:                   return "UNKNOWN";
    }
}

const char* nav_fault_name(NavFault f)
{
    switch (f) {
        case NAV_FAULT_NONE:       return "NONE";
        case NAV_FAULT_ESTOP:      return "ESTOP";
        case NAV_FAULT_MOTOR_LOST: return "MOTOR_LOST";
        case NAV_FAULT_LINE_LOST:  return "LINE_LOST";
        case NAV_FAULT_TIMEOUT:    return "TIMEOUT";
        case NAV_FAULT_IMU:        return "IMU";
        default:                   return "UNKNOWN";
    }
}
