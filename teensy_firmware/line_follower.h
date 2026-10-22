#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
//  8-Kanal QTR-8A Çizgi Takip Modülü
//  Pololu QTR-8A analog reflektans sensör dizisi.
//  Ağırlıklı ortalama + PID kontrol ile steer çıktısı üretir.
//
//  Sensör dizilimi (yere bakıyor):
//    S0(en sol) — S1 — S2 — S3 — S4 — S5 — S6 — S7(en sağ)
//    Ağırlıklar: -3.5  -2.5  -1.5  -0.5  +0.5  +1.5  +2.5  +3.5
// ============================================================================

typedef enum {
    LINE_STATE_ON_LINE,       // Çizgi üzerinde takip
    LINE_STATE_LOST_LEFT,     // Çizgi soldan kayboldu
    LINE_STATE_LOST_RIGHT,    // Çizgi sağdan kayboldu
    LINE_STATE_LOST_ALL,      // Çizgi tamamen kayıp
    LINE_STATE_CROSS          // Kavşak algılandı (çoğu sensör aktif)
} LineState;

typedef struct {
    int       raw[LINE_SENSOR_COUNT]; // Ham ADC değerleri (8 kanal)
    bool      active[LINE_SENSOR_COUNT]; // Eşik üzeri aktifler
    float     position;               // Çizgi pozisyonu (-3.5 ↔ +3.5)
    int       active_count;           // Aktif sensör sayısı
    LineState state;
    float     steer_output;           // PID çıktısı (steer)
    uint32_t  last_seen_ms;           // Çizginin son görüldüğü zaman
} LineData;

// --- API ---
void        line_init(void);
void        line_update(void);
LineData*   line_get_data(void);
float       line_get_steer(void);
bool        line_is_on_line(void);
bool        line_is_cross(void);
const char* line_state_name(LineState s);

#endif // LINE_FOLLOWER_H
