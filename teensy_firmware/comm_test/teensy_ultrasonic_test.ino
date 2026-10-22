#define US_FRONT_TRIG  24
#define US_FRONT_ECHO  25
#define US_LEFT_TRIG   26
#define US_LEFT_ECHO   27
#define US_RIGHT_TRIG  28
#define US_RIGHT_ECHO  29

#define US_TIMEOUT_US  12000 

void setup() {
    Serial.begin(115200);

    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);

    pinMode(13, OUTPUT); // Dahili LED

    while (!Serial && millis() < 3000) {}

    Serial.println("=====================================");
    Serial.println("HC-SR04 ULTRASONIK TEST (ON, SOL, SAG)");
    Serial.println("=====================================");
}

float read_us(int trigPin, int echoPin) {
    digitalWriteFast(trigPin, LOW);
    delayMicroseconds(2);
    digitalWriteFast(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWriteFast(trigPin, LOW);

    // pulseIn komutu ses dalgasinin gidis-gelis suresini mikrosaniye olarak verir
    long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);

    if (duration == 0) {
        return -1.0; // Olcum alinamadi veya 200cm'den uzak
    }
    return (duration / 2.0) * 0.0343; // Ses hizi carpanina gore CM hesapla
}

void loop() {
    digitalWrite(13, HIGH);

    float front_cm = read_us(US_FRONT_TRIG, US_FRONT_ECHO);
    delay(20); // Sensorler arasi kisa bekleme (ciftlesme yansimasini onler)

    float left_cm = read_us(US_LEFT_TRIG, US_LEFT_ECHO);
    delay(20);

    float right_cm = read_us(US_RIGHT_TRIG, US_RIGHT_ECHO);

    digitalWrite(13, LOW);

    // Sonuclari yazdir
    Serial.print("[ON]: ");
    if(front_cm > 0) { Serial.print(front_cm, 1); Serial.print(" cm"); } else { Serial.print("OUT OF RANGE"); }

    Serial.print("  |  [SOL]: ");
    if(left_cm > 0) { Serial.print(left_cm, 1); Serial.print(" cm"); } else { Serial.print("OUT OF RANGE"); }

    Serial.print("  |  [SAG]: ");
    if(right_cm > 0) { Serial.print(right_cm, 1); Serial.println(" cm"); } else { Serial.println("OUT OF RANGE"); }

    delay(200); // Ekrani yormamak icin 5 Hz ile okuma (Test amacli yavaslatildi)
}
