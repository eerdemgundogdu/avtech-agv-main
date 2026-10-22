// ============================================================================
// AVTECH AGV - STEP MOTOR (ASANSOR/FORKLIFT) TEST KODU
//
// Bu kod, ana yazilimimizdaki (navigation.cpp) step motor mantiginin
// %100 BİREBİR aynisidir (PUL, DIR, ENA mantiklari, bekleme sureleri).
// Eger bu kod ile mekanizma sorunsuz calisiyorsa, sistem yuzde yuz hazirdir.
//
// KULLANIM:
// 1. Kodu Teensy 4.1'e yukleyin.
// 2. Arduino IDE "Seri Port Ekrani"ni (Serial Monitor) acin.
// 3. Sag alttan "No line ending" (Satis sonu yok) ve "115200 baud" secin.
// 4. Ekrana asagidaki harfleri yazip Enter'a basin:
//    'w' -> Yuku KALDIR (800 adim - Yarim Tur)
//    's' -> Yuku BIRAK (800 adim - Yarim Tur)
//    'e' -> Motoru ENABLE (Kilitli) yap
//    'd' -> Motoru DISABLE (Serbest) yap
// ============================================================================

#define STEPPER_STEP_PIN        33        // PUL
#define STEPPER_DIR_PIN         34        // DIR
#define STEPPER_EN_PIN          35        // ENA

#define STEPPER_PULSE_US        5         // Ana koddaki mikrosaniye degeri
#define STEPPER_PULSE_DELAY_US  500       // Ana koddaki hiz belirleyici delay

// Hareket edilecek adim sayilari
#define GRIPPER_STEPS           800       // Yarim tur
#define DIR_UP                  HIGH
#define DIR_DOWN                LOW

void stepper_enable() {
    digitalWrite(STEPPER_EN_PIN, LOW); // LOW = aktif (Optokuplor kapali -> motor kilitli)
    delay(1);
    Serial.println("[MOTOR] ENABLED (Kilitlendi)");
}

void stepper_disable() {
    digitalWrite(STEPPER_EN_PIN, HIGH); // HIGH = pasif (Optokuplor acik -> motor serbest)
    delay(1);
    Serial.println("[MOTOR] DISABLED (Serbest Birakildi)");
}

void stepper_move(int steps, bool dir) {
    // 1. Motoru aktif et ve Yönü belirle
    stepper_enable();
    digitalWrite(STEPPER_DIR_PIN, dir);
    delay(1); // Yönün surucude oturmasi icin kucuk bekleme

    Serial.print("[HAREKET] Yon: ");
    Serial.print(dir == DIR_UP ? "YUKARI" : "ASAGI");
    Serial.print(" | Adim Sayisi: ");
    Serial.println(steps);

    // 2. Adimlari ana kodla BİREBİR ayni zamanlamayla at
    for(int i = 0; i < steps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(STEPPER_PULSE_US);       // 5us HIGH
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(STEPPER_PULSE_DELAY_US); // 500us LOW
    }

    Serial.println("[HAREKET] Tamamlandi.");
    // Guc tasarrufu veya guvenlik icin hareket bitince motoru serbest de birakabilirsiniz, 
    // ancak su anki ana kodda yuk dusmesin diye kilitli (ENABLE) birakiyoruz.
}

void setup() {
    Serial.begin(115200);

    // Pin atamalari
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    pinMode(13, OUTPUT); // Teensy dahili LED
    digitalWrite(13, LOW);

    // Baslangic durumlari (Ana koddaki gibi)
    digitalWrite(STEPPER_STEP_PIN, LOW);
    digitalWrite(STEPPER_DIR_PIN, LOW);
    stepper_disable(); // Ilk acilista serbest olsun

    while (!Serial && millis() < 3000) {}

    Serial.println("=====================================");
    Serial.println("STEP MOTOR (DM556) TEST EKRANI");
    Serial.println("=====================================");
    Serial.println("Komutlar: 'w' (YUKARI), 's' (ASAGI), 'e' (ENABLE), 'd' (DISABLE)");
    Serial.println("Pinler: PUL=33, DIR=34, ENA=35");
}

void loop() {
    if (Serial.available() > 0) {
        char in = Serial.read();

        if (in == 'w') {
            stepper_move(GRIPPER_STEPS, DIR_UP);
        } 
        else if (in == 's') {
            stepper_move(GRIPPER_STEPS, DIR_DOWN);
        }
        else if (in == 'e') {
            stepper_enable();
        }
        else if (in == 'd') {
            stepper_disable();
        }
    }

    // Teensy Uzerindeki LED'i Yak/Sondur (Kalp Atisi / Heartbeat)
    // Saniyede 1 kez yanip sonerek Teensy'nin donmadigini gosterir.
    static uint32_t last_led_ms = 0;
    if (millis() - last_led_ms >= 500) {
        digitalWrite(13, !digitalRead(13));
        last_led_ms = millis();
    }
}
