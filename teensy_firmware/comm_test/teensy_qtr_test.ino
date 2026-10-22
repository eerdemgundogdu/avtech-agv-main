

const int qtrPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int qtrValues[8];

const int threshold = 500; 

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < 8; i++) {
        pinMode(qtrPins[i], INPUT);
    }

    while (!Serial && millis() < 3000) {}

    Serial.println("=====================================");
    Serial.println("QTR-8A ANALOG CIZGI SENSORU TEST EKRANI");
    Serial.println("=====================================");
    Serial.println("Duzen: S0(En Sol)  <------>  S7(En Sag)");
}

void loop() {
    for (int i = 0; i < 8; i++) {
        qtrValues[i] = analogRead(qtrPins[i]);
    }
    Serial.print("HAM: ");
    for (int i = 0; i < 8; i++) {
        Serial.print(qtrValues[i]);
        Serial.print("\t");
    }
    Serial.println();
    Serial.print("MAP: ");
    for (int i = 0; i < 8; i++) {
        if (qtrValues[i] > threshold) {
            Serial.print("-"); // Beyaz veya zemin
        } else {
            Serial.print("O"); // Siyah cizgi
        }
        Serial.print("\t");
    }
    Serial.println("\n");

    delay(250);
}
