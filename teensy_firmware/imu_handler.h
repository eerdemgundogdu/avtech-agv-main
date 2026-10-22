#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>
#include <SPI.h>
#include "config.h"

// ============================================================================
//  IMU Handler — ICM-20948 (SPI Bağlantı)
//  Complementary filter ile yaw/pitch/roll hesaplama.
//  Heading hold PID kontrolü.
//
//  ICM-20948 SPI Bağlantısı (Teensy 4.1 SPI0):
//    SCK  → Pin 13
//    MOSI → Pin 11
//    MISO → Pin 12
//    CS   → Pin 10 (config.h: IMU_CS_PIN)
//
//  SPI Modu: Mode 0 (CPOL=0, CPHA=0), MSB first
//  Okuma: Bit7=1 (read), Bit7=0 (write)
// ============================================================================

typedef struct {
    float yaw;         // Derece (0-360)
    float pitch;       // Derece
    float roll;        // Derece
    float gyro_z;      // °/s (dönüş hızı)
    float accel_x;     // m/s² (ileri-geri)
    float accel_y;     // m/s² (sağ-sol)
    bool  healthy;
    uint32_t last_read_ms;
} IMU_Data_AGV;

// --- API ---
bool    imu_agv_init(void);
bool    imu_agv_read(IMU_Data_AGV *data);
bool    imu_agv_is_healthy(void);
void    imu_set_yaw(float absolute_yaw);

// Heading hold
float   imu_heading_hold(float target_yaw, float current_yaw, float dt);
void    imu_heading_reset_pid(void);

// Yardımcı
float   imu_angle_diff(float target, float current);

#endif // IMU_HANDLER_H
