#ifndef HOVER_PROTOCOL_H
#define HOVER_PROTOCOL_H

// ============================================================================
//  Hoverboard FOC USART protokolu — cerceve kodlama/cozme ve rampa
//  Referans: https://github.com/EFeru/hoverboard-firmware-hack-FOC
//
//  Arduino'ya bagimli degil: host'ta firmware_tests/ ile test edilir.
//  Cerceveler little-endian 16-bit kelimelerden olusur; checksum, son kelime
//  haric tum kelimelerin XOR'udur.
//
//  Komut  (Teensy -> Hoverboard),  8 bayt: start, steer, speed, checksum
//  Geri bildirim (Hoverboard -> Teensy), 18 bayt:
//      start, cmd1, cmd2, speedR_meas, speedL_meas, batVoltage,
//      boardTemp, cmdLed, checksum
// ============================================================================

#include <stdint.h>
#include <stddef.h>

#define HOVER_PROTO_START       0xABCD
#define HOVER_CMD_LEN           8
#define HOVER_FEEDBACK_LEN      18

typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} HoverCommand;

typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;    // x100 V
    int16_t  boardTemp;     // x10 °C
    uint16_t cmdLed;
    uint16_t checksum;
} HoverFeedback;

// Little-endian 16-bit kelimelerin XOR'u.
uint16_t hover_checksum(const uint8_t *bytes, size_t word_count);

void hover_encode_command(int16_t steer, int16_t speed, uint8_t out[HOVER_CMD_LEN]);

// Bayt bayt beslenen geri bildirim cozucusu (tahsis yok, hizalama varsayimi yok).
typedef struct {
    uint8_t  buf[HOVER_FEEDBACK_LEN];
    uint8_t  idx;
    uint32_t frames_ok;
    uint32_t checksum_errors;
} HoverRx;

typedef enum {
    HOVER_RX_NONE = 0,
    HOVER_RX_FRAME,         // gecerli cerceve; out dolduruldu
    HOVER_RX_BAD_CHECKSUM,  // tam cerceve geldi ama checksum tutmadi
} HoverRxResult;

void          hover_rx_init(HoverRx *rx);
HoverRxResult hover_rx_feed(HoverRx *rx, uint8_t byte, HoverFeedback *out);

// --- Rampa (yumusak hiz gecisi) ----------------------------------------------

typedef struct {
    int16_t  current;
    int16_t  target;
    uint32_t last_ramp_ms;
} RampState;

// current'i target'a en fazla `step` kadar yaklastirir; target 0 ise
// (durdurma) adim iki katina cikar. Yeni current degerini dondurur.
int16_t hover_ramp_step(RampState *r, int16_t step);

#endif // HOVER_PROTOCOL_H
