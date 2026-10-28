// ============================================================================
//  AVTECH AGV — Ana Teensy 4.1 Firmware
//  Yarışma: Sanayide Robotik Uygulamalar — Hareket-Kabiliyet Videosu
//
//  Donanım:  Teensy 4.1 + Hoverboard FOC (VARIANT_USART, SPD_MODE, tek UART)
//  Motorlar: 2x Hoverboard BLDC @24V
//  Sensörler: ICM-20948 IMU (SPI), QTR-8A + 3x HC-SR04 (MANUAL_ONLY_MODE=0 ise)
//  Aktüatör: DM556 Step Motor Driver (forklift asansörü, konum takipli)
//
//  İletişim:
//    Serial  (USB)      → Jetson Orin Nano (JSON satırları, 115200)
//    Serial1 (RX0/TX1)  → Hoverboard FOC (speed + steer çerçevesi, 0xABCD)
//    SPI0 (10/11/12/13) → ICM-20948 IMU
//
//  Jetson → Teensy komutları (her satır bir JSON nesnesi, boşluklu/boşluksuz kabul):
//    {"cmd":"hb"}                                  kalp atışı (5 Hz, yanıt yok)
//    {"cmd":"manual","speed":S,"steer":T}           manuel sürüş (500 ms keepalive)
//    {"cmd":"stop"} / {"cmd":"reset"} / {"cmd":"ping"}
//    {"cmd":"forklift","action":"up"|"down"|"stop"|"zero"}
//    {"cmd":"forklift","action":"jog","steps":N[,"nolimit":1]}
//    {"cmd":"forklift","action":"goto","pos":N}
//    {"cmd":"forklift","action":"hold","dir":1|-1}   basili tut (100 ms keepalive, FORK_HOLD_TIMEOUT_MS)
//    {"cmd":"hold","on":0|1}                         duz gidiste IMU yon tutma ac/kapa
//    {"cmd":"sync_yaw","yaw":Y}
//    Otonom: {"cmd":"turn"} / "forward" / "start" / "slow_forward" / "fused_line"
//            (MANUAL_ONLY_MODE=1 iken reddedilir veya yok sayilir)
// ============================================================================

#include <SPI.h>
#include "config.h"
#include "hoverboard_driver.h"
#include "line_follower.h"
#include "obstacle_detector.h"
#include "imu_handler.h"
#include "navigation.h"
#include "cmd_parser.h"

// --- Global State ---

static IMU_Data_AGV   imu_data;
static bool           imu_ok = false;
static uint32_t       last_hover_send_ms = 0;
static uint32_t       last_telemetry_ms  = 0;
static uint32_t       last_debug_ms      = 0;
static uint32_t       last_imu_ms        = 0;

// Jetson bağlantı izleme
static uint32_t       last_jetson_rx_ms  = 0;
static bool           hb_seen            = false;   // İlk hb gelince watchdog devreye girer
static bool           link_lost_latched  = false;

// Jetson komut satiri birlestirici (cmd_parser.cpp)
static LineAssembler  cmd_line;

// --- JSON Ayrıştırma ---
// Anahtar okuyucu ve satir birlestirici cmd_parser.cpp'de (host'ta test ediliyor).
// Python json.dumps varsayılanı '"cmd": "manual"' (iki noktadan sonra boşluk)
// üretir; ayrıştırıcı boşluklara toleranslıdır.

static void process_jetson_command(const char *json)
{
    char cmd_str[32];
    if (!cmd_json_get(json, "cmd", cmd_str, sizeof(cmd_str))) {
        return;
    }

    if (strcmp(cmd_str, "hb") == 0) {
        if (!hb_seen) Serial.println("[SYS] Jetson kalp atisi alindi — link watchdog aktif");
        hb_seen = true;
    }
    else if (strcmp(cmd_str, "manual") == 0) {
        long spd = 0, str = 0;
        cmd_json_get_long(json, "speed", &spd);
        cmd_json_get_long(json, "steer", &str);
        spd = constrain(spd, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
        str = constrain(str, -MOTOR_STEER_MAX, MOTOR_STEER_MAX);
        nav_cmd_manual((int16_t)spd, (int16_t)str);
    }
    else if (strcmp(cmd_str, "stop") == 0) {
        nav_cmd_stop();
    }
    else if (strcmp(cmd_str, "reset") == 0) {
        nav_reset();
    }
    else if (strcmp(cmd_str, "ping") == 0) {
        JETSON_SERIAL.println("{\"resp\":\"pong\"}");
    }
    else if (strcmp(cmd_str, "forklift") == 0) {
        char action[16];
        if (!cmd_json_get(json, "action", action, sizeof(action))) return;
        if (strcmp(action, "up") == 0) {
            nav_cmd_forklift(true);
        } else if (strcmp(action, "down") == 0) {
            nav_cmd_forklift(false);
        } else if (strcmp(action, "stop") == 0) {
            nav_cmd_fork_stop();
        } else if (strcmp(action, "zero") == 0) {
            nav_cmd_fork_zero();
        } else if (strcmp(action, "jog") == 0) {
            long steps = 0, nolimit = 0;
            if (cmd_json_get_long(json, "steps", &steps) && steps != 0) {
                cmd_json_get_long(json, "nolimit", &nolimit);
                nav_cmd_fork_jog((int32_t)steps, nolimit != 0);
            }
        } else if (strcmp(action, "goto") == 0) {
            long pos = 0;
            if (cmd_json_get_long(json, "pos", &pos)) {
                nav_cmd_fork_goto((int32_t)pos);
            }
        } else if (strcmp(action, "hold") == 0) {
            long dir = 0;
            cmd_json_get_long(json, "dir", &dir);
            nav_cmd_fork_hold((int8_t)constrain(dir, -1, 1));
        }
    }
    else if (strcmp(cmd_str, "hold") == 0) {
        long on = 1;
        cmd_json_get_long(json, "on", &on);
        nav_cmd_hold_enable(on != 0);
    }
    else if (strcmp(cmd_str, "start") == 0) {
        nav_cmd_start();
    }
    else if (strcmp(cmd_str, "turn") == 0) {
        float angle;
        if (cmd_json_get_float(json, "angle", &angle)) {
            nav_cmd_turn(constrain(angle, -360.0f, 360.0f));
        }
    }
    else if (strcmp(cmd_str, "forward") == 0) {
        long dur;
        if (cmd_json_get_long(json, "duration", &dur)) {
            nav_cmd_forward((int)dur);
        }
    }
    else if (strcmp(cmd_str, "fused_line") == 0) {
        float offset = 0.0f;
        char source[16] = "";
        cmd_json_get_float(json, "offset", &offset);
        cmd_json_get(json, "source", source, sizeof(source));
        nav_set_fused_line(offset, source);
    }
    else if (strcmp(cmd_str, "sync_yaw") == 0) {
        float yaw;
        if (cmd_json_get_float(json, "yaw", &yaw)) {
            imu_set_yaw(yaw);
        }
    }
    else if (strcmp(cmd_str, "slow_forward") == 0) {
        long spd = 80;
        cmd_json_get_long(json, "speed", &spd);
        nav_cmd_slow_forward((int16_t)constrain(spd, 0, MOTOR_SPEED_MAX));
    }
}

static void read_jetson_serial(uint32_t now)
{
    while (JETSON_SERIAL.available()) {
        // Taşan satır tamamen atılır (kesik komut çalıştırılmaz)
        if (line_assembler_feed(&cmd_line, (char)JETSON_SERIAL.read())) {
            last_jetson_rx_ms = now;
            process_jetson_command(cmd_line.buf);
        }
    }
}

static bool jetson_link_ok(uint32_t now)
{
    return !hb_seen || (now - last_jetson_rx_ms) <= JETSON_LINK_TIMEOUT_MS;
}

// --- Telemetri Gönder ---

static void send_telemetry(uint32_t now)
{
    const MotorStatus *ml = hoverboard_get_left();
    const MotorStatus *mr = hoverboard_get_right();

    JETSON_SERIAL.print("{\"state\":\"");
    JETSON_SERIAL.print(nav_state_name(nav_get_state()));
    JETSON_SERIAL.print("\"");

    // Motor verileri
    JETSON_SERIAL.print(",\"spdL\":");
    JETSON_SERIAL.print(ml->speed_meas);
    JETSON_SERIAL.print(",\"spdR\":");
    JETSON_SERIAL.print(mr->speed_meas);

    // Komut edilen çıkış (ramp öncesi hedef)
    MotorOutput mo = nav_get_output();
    JETSON_SERIAL.print(",\"cmdSpd\":");
    JETSON_SERIAL.print(mo.speed);
    JETSON_SERIAL.print(",\"cmdStr\":");
    JETSON_SERIAL.print(mo.steer);

    // Batarya
    JETSON_SERIAL.print(",\"bat\":");
    JETSON_SERIAL.print(hoverboard_get_battery_voltage(), 1);

    // IMU
    JETSON_SERIAL.print(",\"imu\":");
    JETSON_SERIAL.print(imu_ok ? "true" : "false");
    JETSON_SERIAL.print(",\"yaw\":");
    JETSON_SERIAL.print(imu_data.yaw, 1);
    JETSON_SERIAL.print(",\"gyroZ\":");
    JETSON_SERIAL.print(imu_data.gyro_z, 1);

    // Yon tutma: holdEn = acik mi, hold = 0 kapali / 1 bekliyor / 2 aktif, tgtYaw = kilitli yon
    JETSON_SERIAL.print(",\"holdEn\":");
    JETSON_SERIAL.print(nav_hold_enabled() ? "true" : "false");
    JETSON_SERIAL.print(",\"hold\":");
    JETSON_SERIAL.print((int)nav_hold_state());
    JETSON_SERIAL.print(",\"tgtYaw\":");
    JETSON_SERIAL.print(nav_hold_target(), 1);

    // Forklift
    JETSON_SERIAL.print(",\"fork\":");
    JETSON_SERIAL.print((long)nav_fork_position());
    JETSON_SERIAL.print(",\"forkTop\":");
    JETSON_SERIAL.print((long)FORK_TOP_STEPS);
    JETSON_SERIAL.print(",\"forkSt\":\"");
    JETSON_SERIAL.print(nav_fork_state_name());
    JETSON_SERIAL.print("\"");
    JETSON_SERIAL.print(",\"forkHold\":");
    JETSON_SERIAL.print(nav_fork_hold_active() ? "true" : "false");

#if !MANUAL_ONLY_MODE
    LineData *line = line_get_data();
    ObstacleData *obs = obstacle_get_data();

    JETSON_SERIAL.print(",\"line\":\"");
    JETSON_SERIAL.print(line_state_name(line->state));
    JETSON_SERIAL.print("\",\"linePos\":");
    JETSON_SERIAL.print(line->position, 2);

    JETSON_SERIAL.print(",\"lineRaw\":[");
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        JETSON_SERIAL.print(line->raw[i]);
        if (i < LINE_SENSOR_COUNT - 1) JETSON_SERIAL.print(",");
    }
    JETSON_SERIAL.print("]");

    JETSON_SERIAL.print(",\"obsFront\":");
    JETSON_SERIAL.print(obs->distance_cm[0], 0);
    JETSON_SERIAL.print(",\"obsLeft\":");
    JETSON_SERIAL.print(obs->distance_cm[1], 0);
    JETSON_SERIAL.print(",\"obsRight\":");
    JETSON_SERIAL.print(obs->distance_cm[2], 0);
    JETSON_SERIAL.print(",\"blocked\":");
    JETSON_SERIAL.print(obstacle_is_blocked() ? "true" : "false");
#endif

    // Güvenlik
    JETSON_SERIAL.print(",\"estop\":");
    JETSON_SERIAL.print(nav_estop_pressed() ? "true" : "false");
    JETSON_SERIAL.print(",\"link\":");
    JETSON_SERIAL.print(jetson_link_ok(now) ? "true" : "false");
    JETSON_SERIAL.print(",\"mode\":\"");
    JETSON_SERIAL.print(MANUAL_ONLY_MODE ? "MANUAL_ONLY" : "AUTO");
    JETSON_SERIAL.print("\"");

    if (nav_get_state() == NAV_FAULT) {
        JETSON_SERIAL.print(",\"fault\":\"");
        JETSON_SERIAL.print(nav_fault_name(nav_get_fault()));
        JETSON_SERIAL.print("\"");
    }

    // Motor bağlantıları ve sıcaklık
    JETSON_SERIAL.print(",\"motL\":");
    JETSON_SERIAL.print(hoverboard_left_connected() ? "true" : "false");
    JETSON_SERIAL.print(",\"motR\":");
    JETSON_SERIAL.print(hoverboard_right_connected() ? "true" : "false");
    JETSON_SERIAL.print(",\"tempL\":");
    JETSON_SERIAL.print((float)ml->board_temp / 10.0f, 1);
    JETSON_SERIAL.print(",\"tempR\":");
    JETSON_SERIAL.print((float)mr->board_temp / 10.0f, 1);

    JETSON_SERIAL.println("}");
}

// --- Setup ---

void setup()
{
    JETSON_SERIAL.begin(JETSON_BAUD);
    while (!JETSON_SERIAL && millis() < 2000) {}

    JETSON_SERIAL.println("========================================");
    JETSON_SERIAL.println("  AVTECH AGV — Teensy 4.1 Firmware v3");
    JETSON_SERIAL.println("  Sanayide Robotik Uygulamalar 2026");
    JETSON_SERIAL.println("  IMU: ICM-20948 (SPI)");
    JETSON_SERIAL.println("  Surus: Hoverboard FOC (Serial1)");
    JETSON_SERIAL.println("  Yuk: DM556 Step Motor (konum takipli)");
    JETSON_SERIAL.println(MANUAL_ONLY_MODE ? "  Mod: MANUAL_ONLY (GUI + kumanda)" : "  Mod: AUTO (cizgi + engel)");
    JETSON_SERIAL.println("========================================");

    analogReadResolution(ADC_RESOLUTION);
    line_assembler_init(&cmd_line);

    pinMode(ESTOP_PIN, ESTOP_PIN_MODE);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

#ifdef MOTOR_EN_PIN
    pinMode(MOTOR_EN_PIN, OUTPUT);
    digitalWrite(MOTOR_EN_PIN, LOW);      // acilis tamamlanana kadar motor gucu kapali
#endif

    hoverboard_init();
#if !MANUAL_ONLY_MODE
    line_init();
    obstacle_init();
#endif

    JETSON_SERIAL.println("[IMU] Initializing ICM-20948 via SPI...");
    memset(&imu_data, 0, sizeof(imu_data));
    imu_ok = imu_agv_init();
    JETSON_SERIAL.println(imu_ok ? "[IMU] ICM-20948 Ready"
                                 : "[IMU] ICM-20948 FAILED — heading hold disabled");

    // Navigasyon FSM (DM556 stepper init + açılış forklift konumlandırma dahil)
    nav_init();

    // Başlangıç bip (3 kez kısa)
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }

    JETSON_SERIAL.println("[SYS] AVTECH AGV ready.");
    JETSON_SERIAL.println("{\"resp\":\"ready\"}");
}

// --- Loop ---

void loop()
{
    uint32_t now = millis();

    // 1) Jetson komutları
    read_jetson_serial(now);

    // 2) Jetson bağlantı watchdog (sadece ilk hb'den sonra)
    if (!jetson_link_ok(now)) {
        if (!link_lost_latched) {
            link_lost_latched = true;
            JETSON_SERIAL.println("{\"status\":\"link_lost\"}");
            nav_link_lost();
        }
    } else if (link_lost_latched) {
        link_lost_latched = false;
        Serial.println("[SYS] Jetson baglantisi geri geldi");
    }

#ifdef MOTOR_EN_PIN
    // 2b) Kontaktor: ariza yok, Jetson baglantisi var ve E-stop basili degilse
    // motor gucu acik. E-stop ayrica donanimda bobini keser; bu sadece ek katman.
    digitalWrite(MOTOR_EN_PIN,
                 (nav_get_state() != NAV_FAULT && jetson_link_ok(now) && !nav_estop_pressed())
                     ? HIGH : LOW);
#endif

    // 3) Hoverboard feedback
    hoverboard_receive();

    // 4) IMU (100 Hz)
    if (imu_ok && (now - last_imu_ms) >= IMU_READ_INTERVAL_MS) {
        imu_ok = imu_agv_read(&imu_data);
        if (!imu_ok) Serial.println("[IMU] Okuma hatasi — heading hold devre disi");
        last_imu_ms = now;
    }

#if !MANUAL_ONLY_MODE
    line_update();
    obstacle_update();
#endif

    // 5) Navigasyon FSM (forklift servis + E-stop dahil)
    nav_imu_feed(imu_data.gyro_z, imu_ok);
    {
        static uint32_t last_nav_ms = 0;
        float dt = (last_nav_ms > 0) ? (float)(now - last_nav_ms) / 1000.0f : 0.01f;
        if (dt > 0.1f) dt = 0.1f;
        if (dt < 0.001f) dt = 0.001f;
        last_nav_ms = now;
        nav_update(imu_data.yaw, dt);
    }

    // 6) Motor komutu (20 Hz) — aktif değilse veya link koptuysa 0
    if ((now - last_hover_send_ms) >= HOVER_SEND_INTERVAL_MS) {
        MotorOutput mo = nav_get_output();
        if (!nav_is_active() || !jetson_link_ok(now) || nav_estop_pressed()) {
            mo.speed = 0;
            mo.steer = 0;
        }
        hoverboard_send(mo.speed, mo.steer);
        hoverboard_update_ramp();   // Fiziksel UART gönderimi ve rampa işlemi
        last_hover_send_ms = now;
    }

    // 7) Telemetri (10 Hz)
    if ((now - last_telemetry_ms) >= TELEMETRY_INTERVAL_MS) {
        send_telemetry(now);
        last_telemetry_ms = now;
    }

    // 8) Durum LED'i
    {
        static uint32_t led_prev_ms = 0;
        static bool led_on = false;
        if (nav_is_done()) {
            digitalWrite(STATUS_LED_PIN, HIGH);
        } else {
            uint32_t interval = 1000;
            if (nav_get_state() == NAV_FAULT) interval = 100;
            else if (nav_is_active())         interval = 250;
            if ((now - led_prev_ms) >= interval) {
                led_on = !led_on;
                digitalWrite(STATUS_LED_PIN, led_on ? HIGH : LOW);
                led_prev_ms = now;
            }
        }
    }

    // 9) Debug log (~2 Hz)
#if DEBUG_ENABLED
    if ((now - last_debug_ms) >= DEBUG_LOG_INTERVAL_MS) {
        Serial.print("[DBG] ST=");
        Serial.print(nav_state_name(nav_get_state()));
        Serial.print(" YAW=");
        Serial.print(imu_data.yaw, 1);
        Serial.print(" FORK=");
        Serial.print((long)nav_fork_position());
        Serial.print("/");
        Serial.print(nav_fork_state_name());
        Serial.print(" BAT=");
        Serial.print(hoverboard_get_battery_voltage(), 1);
        Serial.print("V MOT=");
        Serial.print(hoverboard_left_connected() ? "OK" : "X");
        Serial.print(" LINK=");
        Serial.println(jetson_link_ok(now) ? (hb_seen ? "OK" : "-") : "LOST");
        last_debug_ms = now;
    }
#endif
}
