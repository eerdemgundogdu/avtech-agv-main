#include "hover_protocol.h"
#include <string.h>

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

uint16_t hover_checksum(const uint8_t *bytes, size_t word_count)
{
    uint16_t c = 0;
    for (size_t i = 0; i < word_count; i++) c ^= get_u16_le(&bytes[i * 2]);
    return c;
}

void hover_encode_command(int16_t steer, int16_t speed, uint8_t out[HOVER_CMD_LEN])
{
    put_u16_le(&out[0], HOVER_PROTO_START);
    put_u16_le(&out[2], (uint16_t)steer);
    put_u16_le(&out[4], (uint16_t)speed);
    put_u16_le(&out[6], hover_checksum(out, 3));
}

void hover_rx_init(HoverRx *rx)
{
    memset(rx, 0, sizeof(*rx));
}

static void decode_feedback(const uint8_t *b, HoverFeedback *out)
{
    out->start       = get_u16_le(&b[0]);
    out->cmd1        = (int16_t)get_u16_le(&b[2]);
    out->cmd2        = (int16_t)get_u16_le(&b[4]);
    out->speedR_meas = (int16_t)get_u16_le(&b[6]);
    out->speedL_meas = (int16_t)get_u16_le(&b[8]);
    out->batVoltage  = (int16_t)get_u16_le(&b[10]);
    out->boardTemp   = (int16_t)get_u16_le(&b[12]);
    out->cmdLed      = get_u16_le(&b[14]);
    out->checksum    = get_u16_le(&b[16]);
}

// Checksum tutmayan 18 baytin icinde baska bir cerceve basi (0xCD 0xAB) varsa
// tamponu oraya kaydirir; yoksa sifirlar. Boylece gurultuyle baslayan sahte
// bir cercevenin icine dusen gercek cerceve kaybolmaz.
static void resync_after_bad_frame(HoverRx *rx, uint8_t lo, uint8_t hi)
{
    for (uint8_t i = 1; i < HOVER_FEEDBACK_LEN; i++) {
        const bool last = (i == HOVER_FEEDBACK_LEN - 1);
        if (rx->buf[i] == lo && (last || rx->buf[i + 1] == hi)) {
            const uint8_t keep = (uint8_t)(HOVER_FEEDBACK_LEN - i);
            memmove(rx->buf, rx->buf + i, keep);
            rx->idx = keep;
            return;
        }
    }
    rx->idx = 0;
}

HoverRxResult hover_rx_feed(HoverRx *rx, uint8_t byte, HoverFeedback *out)
{
    const uint8_t lo = (uint8_t)(HOVER_PROTO_START & 0xFF);   // once 0xCD
    const uint8_t hi = (uint8_t)(HOVER_PROTO_START >> 8);     // sonra 0xAB

    if (rx->idx == 0) {
        if (byte == lo) rx->buf[rx->idx++] = byte;
        return HOVER_RX_NONE;
    }
    if (rx->idx == 1) {
        if (byte == hi) {
            rx->buf[rx->idx++] = byte;
        } else {
            // 0xCD 0xCD 0xAB ...: ikinci 0xCD yeni baslangic olabilir.
            rx->idx = (byte == lo) ? 1 : 0;
        }
        return HOVER_RX_NONE;
    }

    rx->buf[rx->idx++] = byte;
    if (rx->idx < HOVER_FEEDBACK_LEN) return HOVER_RX_NONE;

    const uint16_t expected = hover_checksum(rx->buf, (HOVER_FEEDBACK_LEN / 2) - 1);
    if (get_u16_le(&rx->buf[HOVER_FEEDBACK_LEN - 2]) != expected) {
        rx->checksum_errors++;
        resync_after_bad_frame(rx, lo, hi);
        return HOVER_RX_BAD_CHECKSUM;
    }
    rx->idx = 0;
    decode_feedback(rx->buf, out);
    rx->frames_ok++;
    return HOVER_RX_FRAME;
}

int16_t hover_ramp_step(RampState *r, int16_t step)
{
    if (r->target == 0) step = (int16_t)(step * 2);   // durdururken 2x hizli
    const int32_t diff = (int32_t)r->target - (int32_t)r->current;
    if (diff > step)       r->current = (int16_t)(r->current + step);
    else if (diff < -step) r->current = (int16_t)(r->current - step);
    else                   r->current = r->target;
    return r->current;
}
