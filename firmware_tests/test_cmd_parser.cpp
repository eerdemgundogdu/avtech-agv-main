// cmd_parser icin host birim testleri.

#include "minitest.h"
#include "cmd_parser.h"

#include <cstring>
#include <string>

namespace {

std::string get(const char *json, const char *key, size_t cap = 32)
{
    char out[64] = {};
    if (!cmd_json_get(json, key, out, cap)) return "<none>";
    return out;
}

// Satiri bayt bayt besler, tamamlanan satirlari dondurur.
std::string feed_lines(LineAssembler &la, const std::string &bytes)
{
    std::string got;
    for (char c : bytes) {
        if (line_assembler_feed(&la, c)) {
            got += la.buf;
            got += '|';
        }
    }
    return got;
}

} // namespace

TEST(reads_compact_and_spaced_json)
{
    // teensy_link.py kompakt, json.dumps varsayilani bosluklu uretir.
    CHECK(get("{\"cmd\":\"manual\",\"speed\":120,\"steer\":-30}", "cmd") == "manual");
    CHECK(get("{\"cmd\": \"manual\", \"speed\": 120, \"steer\": -30}", "steer") == "-30");
    CHECK(get("{ \"cmd\" :\t\"stop\" }", "cmd") == "stop");
}

TEST(key_text_inside_a_value_is_not_a_key)
{
    // "source" degeri "cmd" metnini iceriyor; anahtar olarak alinmamali.
    const char *j = "{\"source\":\"cmd\",\"cmd\":\"fused_line\"}";
    CHECK(get(j, "cmd") == "fused_line");
}

TEST(missing_or_empty_values)
{
    CHECK(get("{\"cmd\":\"ping\"}", "speed") == "<none>");
    CHECK(get("{\"speed\":}", "speed") == "<none>");
    CHECK(get("{\"cmd\":\"\"}", "cmd") == "<none>");
}

TEST(quoted_value_too_long_is_rejected_not_truncated)
{
    // Kesik bir komut adi ("forkl") yanlis bir komutla eslesebilir.
    CHECK(get("{\"cmd\":\"forklift\"}", "cmd", 6) == "<none>");
    CHECK(get("{\"cmd\":\"forklift\"}", "cmd", 9) == "forklift");
}

TEST(numbers)
{
    long v = 0;
    CHECK(cmd_json_get_long("{\"steps\":-1500}", "steps", &v));
    CHECK_EQ(v, -1500);
    CHECK(!cmd_json_get_long("{\"steps\":\"abc\"}", "steps", &v));

    float f = 0.0f;
    CHECK(cmd_json_get_float("{\"angle\": 90.5}", "angle", &f));
    CHECK(f > 90.49f && f < 90.51f);
    CHECK(!cmd_json_get_float("{\"angle\":nan}", "angle", &f));
    CHECK(!cmd_json_get_float("{\"angle\":inf}", "angle", &f));
}

TEST(key_longer_than_pattern_buffer_is_rejected)
{
    char out[8];
    const std::string key(64, 'k');
    CHECK(!cmd_json_get("{}", key.c_str(), out, sizeof(out)));
}

TEST(line_assembler_splits_lines_and_skips_empty)
{
    LineAssembler la;
    line_assembler_init(&la);
    CHECK(feed_lines(la, "{\"cmd\":\"hb\"}\r\n\n{\"cmd\":\"stop\"}\n") ==
          "{\"cmd\":\"hb\"}|{\"cmd\":\"stop\"}|");
}

TEST(overlong_line_is_dropped_entirely)
{
    // Tampona sigmayan satirin kesik hali calistirilmamali; sonraki satir
    // normal islenmeli.
    LineAssembler la;
    line_assembler_init(&la);
    std::string big = "{\"cmd\":\"manual\",\"pad\":\"" + std::string(400, 'x') + "\"}\n";
    CHECK(feed_lines(la, big + "{\"cmd\":\"stop\"}\n") == "{\"cmd\":\"stop\"}|");
    CHECK_EQ(la.lines_dropped, 1);
}

TEST(line_exactly_at_capacity)
{
    LineAssembler la;
    line_assembler_init(&la);
    const std::string fits(CMD_LINE_MAX - 1, 'a');
    CHECK(feed_lines(la, fits + "\n") == fits + "|");
    const std::string too_long(CMD_LINE_MAX, 'a');
    CHECK(feed_lines(la, too_long + "\n") == "");
}

TEST_MAIN()
