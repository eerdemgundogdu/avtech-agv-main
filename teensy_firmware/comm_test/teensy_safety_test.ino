
#define ESTOP_PIN       36
#define BUZZER_PIN      37
#define STATUS_LED_PIN  2

void setup() {
    Serial.begin(115200);
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    while (!Serial && millis() < 3000) {}

    Serial.println("=====================================");
    Serial.println("GUVENLIK DONANIMLARI TEST EKRANI");
    Serial.println("=====================================");
    Serial.println("E-STOP butonuna basin (Pin 36'yi GND'ye temas ettirin).");
}

void loop() {
    // E-Stop butonu aktif oldugunda pini LOW (0V) yapar
    bool is_estop_pressed = (digitalRead(ESTOP_PIN) == LOW);

    if (is_estop_pressed) {
        Serial.println("[ACIL DURDURMA] Butona basildi! Sistem Guvenli Moda alindi.");

        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(STATUS_LED_PIN, HIGH);

        delay(500); 
    } else {
        digitalWrite(BUZZER_PIN, LOW);
        if ((millis() / 500) % 2 == 0) {
            digitalWrite(STATUS_LED_PIN, HIGH);
        } else {
            digitalWrite(STATUS_LED_PIN, LOW);
        }
    }
}
