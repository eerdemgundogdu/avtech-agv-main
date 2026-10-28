// hover_protocol icin host birim testleri.

#include "minitest.h"
#include "hover_protocol.h"

#include <random>
#include <vector>

namespace {

// EFeru FOC firmware'inin geri bildirim cercevesini uretir (little-endian).
std::vector<uint8_t> make_feedback(int16_t cmd1, int16_t cmd2, int16_t spdR,
                                   int16_t spdL, int16_t bat, int16_t temp)
{
    const uint16_t w[8] = {HOVER_PROTO_START, (uint16_t)cmd1, (uint16_t)cmd2,
                           (uint16_t)spdR, (uint16_t)spdL, (uint16_t)bat,
                           (uint16_t)temp, 0};
    uint16_t cs = 0;
    std::vector<uint8_t> f;
    for (uint16_t v : w) {
        cs ^= v;
        f.push_back((uint8_t)(v & 0xFF));
        f.push_back((uint8_t)(v >> 8));
    }
    f.push_back((uint8_t)(cs & 0xFF));
    f.push_back((uint8_t)(cs >> 8));
    return f;
}

struct Counts { int ok = 0, bad = 0; HoverFeedback last{}; };

Counts feed(HoverRx &rx, const std::vector<uint8_t> &bytes)
{
    Counts c;
    HoverFeedback fb;
    for (uint8_t b : bytes) {
        switch (hover_rx_feed(&rx, b, &fb)) {
        case HOVER_RX_FRAME: c.ok++; c.last = fb; break;
        case HOVER_RX_BAD_CHECKSUM: c.bad++; break;
        default: break;
        }
    }
    return c;
}

} // namespace

TEST(struct_sizes_match_wire_format)
{
    CHECK_EQ(sizeof(HoverCommand), HOVER_CMD_LEN);
    CHECK_EQ(sizeof(HoverFeedback), HOVER_FEEDBACK_LEN);
}

TEST(command_frame_golden_bytes)
{
    uint8_t f[HOVER_CMD_LEN];
    hover_encode_command(0, 0, f);
    const uint8_t zero[HOVER_CMD_LEN] = {0xCD, 0xAB, 0, 0, 0, 0, 0xCD, 0xAB};
    for (int i = 0; i < HOVER_CMD_LEN; i++) CHECK_EQ(f[i], zero[i]);

    // steer = 100 (0x0064), speed = -200 (0xFF38)
    // checksum = 0xABCD ^ 0x0064 ^ 0xFF38 = 0x5491
    hover_encode_command(100, -200, f);
    const uint8_t mixed[HOVER_CMD_LEN] = {0xCD, 0xAB, 0x64, 0x00, 0x38, 0xFF, 0x91, 0x54};
    for (int i = 0; i < HOVER_CMD_LEN; i++) CHECK_EQ(f[i], mixed[i]);
}

TEST(feedback_decodes_signed_fields)
{
    HoverRx rx;
    hover_rx_init(&rx);
    Counts c = feed(rx, make_feedback(120, -120, -350, 349, 2412, 385));
    CHECK_EQ(c.ok, 1);
    CHECK_EQ(c.last.cmd2, -120);
    CHECK_EQ(c.last.speedR_meas, -350);
    CHECK_EQ(c.last.speedL_meas, 349);
    CHECK_EQ(c.last.batVoltage, 2412);
    CHECK_EQ(c.last.boardTemp, 385);
}

TEST(bad_checksum_is_counted_and_next_frame_parses)
{
    HoverRx rx;
    hover_rx_init(&rx);
    std::vector<uint8_t> s = make_feedback(1, 2, 3, 4, 2400, 300);
    s[9] ^= 0x10;
    std::vector<uint8_t> good = make_feedback(5, 6, 7, 8, 2401, 301);
    s.insert(s.end(), good.begin(), good.end());
    Counts c = feed(rx, s);
    CHECK_EQ(c.bad, 1);
    CHECK_EQ(c.ok, 1);
    CHECK_EQ(c.last.speedL_meas, 8);
    CHECK_EQ(rx.checksum_errors, 1);
}

TEST(repeated_start_low_byte_resyncs)
{
    // 0xCD 0xCD 0xAB ...: ilk 0xCD gurultu, ikincisi cerceve basi.
    HoverRx rx;
    hover_rx_init(&rx);
    std::vector<uint8_t> s = {0x00, 0xCD};
    std::vector<uint8_t> f = make_feedback(0, 0, 10, 20, 2400, 250);
    s.insert(s.end(), f.begin(), f.end());
    CHECK_EQ(feed(rx, s).ok, 1);
}

TEST(random_noise_never_produces_a_frame)
{
    // Gurultudeki bir 0xCD sahte bir cerceve baslatir ve sonraki gercek
    // cercevenin baytlarini yutar; checksum tutmayinca cozucu yutulan
    // baytlardaki gercek basliga kaymali. Sabit tohumla: sahte cerceve yok,
    // gercek cercevelerin hepsi yakalaniyor.
    std::mt19937 rng(3);
    std::uniform_int_distribution<int> byte(0, 255), gap(0, 12);
    HoverRx rx;
    hover_rx_init(&rx);
    int real = 0, got = 0, bogus = 0;
    HoverFeedback fb;
    for (int k = 0; k < 1000; k++) {
        for (int i = gap(rng); i > 0; i--) {
            if (hover_rx_feed(&rx, (uint8_t)byte(rng), &fb) == HOVER_RX_FRAME) bogus++;
        }
        for (uint8_t b : make_feedback(1, 2, 3, (int16_t)k, 2400, 300)) {
            if (hover_rx_feed(&rx, b, &fb) == HOVER_RX_FRAME) {
                if (fb.speedL_meas == k) got++; else bogus++;
            }
        }
        real++;
    }
    CHECK_EQ(bogus, 0);
    CHECK_EQ(got, real);
}

TEST(real_frame_inside_a_false_start_is_recovered)
{
    // Tek bir 0xCD gurultu bayti + 0xAB ile "cerceve basi" gibi gorunur;
    // ardindan gelen gercek cerceve bu sahte cercevenin icine duser.
    HoverRx rx;
    hover_rx_init(&rx);
    std::vector<uint8_t> s = {0xCD, 0xAB, 0x01, 0x02, 0x03};
    std::vector<uint8_t> f = make_feedback(9, 9, 9, 42, 2400, 300);
    s.insert(s.end(), f.begin(), f.end());
    Counts c = feed(rx, s);
    CHECK_EQ(c.ok, 1);
    CHECK_EQ(c.last.speedL_meas, 42);
}

TEST(ramp_limits_step_and_stops_twice_as_fast)
{
    RampState r = {0, 100, 0};
    CHECK_EQ(hover_ramp_step(&r, 10), 10);
    CHECK_EQ(hover_ramp_step(&r, 10), 20);
    r.current = 95;
    CHECK_EQ(hover_ramp_step(&r, 10), 100);   // hedefi gecmez
    r.target = 0;
    CHECK_EQ(hover_ramp_step(&r, 10), 80);    // durdurmada 2x adim
}

TEST(ramp_full_reverse_does_not_overflow)
{
    RampState r = {400, -400, 0};
    int steps = 0;
    while (r.current != r.target && steps < 1000) { hover_ramp_step(&r, 10); steps++; }
    CHECK_EQ(r.current, -400);
    CHECK_EQ(steps, 80);
}

TEST_MAIN()
