

#include <Arduino.h>
#include <SPI.h>


#include "../config.h"
#include "../imu_handler.cpp" 

IMU_Data_AGV imu_data;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("=====================================");
    Serial.println("ICM-20948 GERCEK ZAMANLI ACI TESTI");
    Serial.println("=====================================");


    if (!imu_agv_init()) {
        Serial.println("[HATA] IMU baslatilamadi! Kablolari kontrol edin (CS=10, MOSI=11, MISO=12, SCK=13)");
        while (1) {
            delay(100); // Sonsuz dongu
        }
    }

    Serial.println("[BASARILI] IMU Hazir! 2 Saniye icinde aci akisi baslayacak...");
    delay(2000);
}

void loop() {
    if (imu_agv_read(&imu_data)) {
        Serial.print("YAW (Saga-Sola Donus): ");
        Serial.print(imu_data.yaw, 2);

        Serial.print("   | PITCH (Yokusa Tirmandigimiz Aci): ");
        Serial.print(imu_data.pitch, 2);

        Serial.print("   | ROLL (Yana Yatmasi): ");
        Serial.print(imu_data.roll, 2);

        Serial.print("   | GYRO_Z (Donus Hizi): ");
        Serial.print(imu_data.gyro_z, 2);
        Serial.println(" deg/s");
    } else {
        Serial.println("[HATA] IMU'dan veri okunamadi!");
    }

    delay(20);
}
