//    'w' -> Ileri
//    's' -> Geri
//    'a' -> Sola Donus
//    'd' -> Saga Donus
//    ' ' (bosluk) -> DUR

#define HOVER_SERIAL Serial1
#define HOVER_BAUD 115200
#define HOVER_START_FRAME 0xABCD

typedef struct {
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} HoverCommand;

typedef struct {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} HoverFeedback;

HoverCommand cmd;
HoverFeedback feedback;
uint32_t last_send_ms = 0;
uint32_t last_print_ms = 0;

int16_t current_speed = 0;
int16_t current_steer = 0;
bool is_connected = false;

uint16_t calc_checksum_words(const uint16_t *words, uint16_t word_count) {
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < word_count; i++) {
        checksum ^= words[i];
    }
    return checksum;
}

void setup() {
    Serial.begin(115200);
    HOVER_SERIAL.begin(HOVER_BAUD);

    while (!Serial && millis() < 3000) {}

    Serial.println("=====================================");
    Serial.println("HOVERBOARD USART TEST (FOC SPD_MODE)");
    Serial.println("=====================================");
    Serial.println("Komutlar: 'w' (Ileri), 's' (Geri), 'a' (Sol), 'd' (Sag), ' ' (DUR)");
    Serial.println("Serial1 (RX1=0, TX1=1) uzerinden motorlara bagli olmali.");
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
}

void loop() {
    if (Serial.available() > 0) {
        char in = Serial.read();
        if (in == 'w') {
            current_speed = 100;
            current_steer = 0;
            Serial.println("[TEST] ILERI (Speed: 100, Steer: 0)");
        } 
        else if (in == 's') {
            current_speed = -100;
            current_steer = 0;
            Serial.println("[TEST] GERI (Speed: -100, Steer: 0)");
        }
        else if (in == 'a') {
            current_speed = 0;
            current_steer = 50;
            Serial.println("[TEST] SOL (Speed: 0, Steer: 50)");
        }
        else if (in == 'd') {
            current_speed = 0;
            current_steer = -50;
            Serial.println("[TEST] SAG (Speed: 0, Steer: -50)");
        }
        else if (in == ' ') {
            current_speed = 0;
            current_steer = 0;
            Serial.println("[TEST] DUR (Speed: 0, Steer: 0)");
        }
    }

    if (millis() - last_send_ms >= 50) {
        cmd.start = HOVER_START_FRAME;
        cmd.steer = current_steer;
        cmd.speed = current_speed;
        cmd.checksum = calc_checksum_words((uint16_t *)&cmd, 3);
        HOVER_SERIAL.write((uint8_t *)&cmd, sizeof(cmd));

        last_send_ms = millis();
    }

    uint8_t *fb_ptr = (uint8_t *)&feedback;
    static uint16_t fb_idx = 0;

    while (HOVER_SERIAL.available() > 0) {
        fb_ptr[fb_idx++] = HOVER_SERIAL.read();

        if (fb_idx == sizeof(HoverFeedback)) {
            uint16_t calc_chk = calc_checksum_words((uint16_t *)&feedback, 8);
            if (feedback.start == HOVER_START_FRAME && calc_chk == feedback.checksum) {
                is_connected = true;
                if (millis() - last_print_ms >= 1000) {
                    Serial.print("[BAGLANTI OK] Batarya: ");
                    Serial.print((float)feedback.batVoltage / 100.0f);
                    Serial.print("V | Sol RPM: ");
                    Serial.print(feedback.speedL_meas);
                    Serial.print(" | Sag RPM: ");
                    Serial.println(feedback.speedR_meas);
                    last_print_ms = millis();
                }
            } else {
                fb_idx = 0;
            }
            fb_idx = 0;
        }
    }
    // Eger motorla baglanti varsa HIZLI (100ms), yoksa YAVAS (1000ms) yanip soner.
    static uint32_t last_led_ms = 0;
    uint32_t blink_interval = is_connected ? 100 : 1000;

    if (millis() - last_led_ms >= blink_interval) {
        digitalWrite(13, !digitalRead(13));
        last_led_ms = millis();
    }
}
