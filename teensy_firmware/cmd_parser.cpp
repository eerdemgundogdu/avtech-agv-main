#include "cmd_parser.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

bool cmd_json_get(const char *json, const char *key, char *out, size_t max_len)
{
    if (max_len == 0) return false;

    char pat[40];
    const int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(pat)) return false;
    const size_t plen = (size_t)n;

    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        const char *q = skip_ws(p + plen);
        if (*q != ':') { p += plen; continue; }   // bu bir deger, anahtar degil
        q = skip_ws(q + 1);

        size_t i = 0;
        if (*q == '"') {
            q++;
            while (*q && *q != '"' && i < max_len - 1) out[i++] = *q++;
            if (*q != '"') { out[0] = '\0'; return false; }   // kesik / cok uzun
        } else {
            while (*q && *q != ',' && *q != '}' && *q != ' ' && *q != '\t' &&
                   i < max_len - 1) {
                out[i++] = *q++;
            }
        }
        out[i] = '\0';
        return i > 0;
    }
    return false;
}

bool cmd_json_get_long(const char *json, const char *key, long *out)
{
    char buf[24];
    if (!cmd_json_get(json, key, buf, sizeof(buf))) return false;
    char *end = NULL;
    const long v = strtol(buf, &end, 10);
    if (end == buf) return false;
    *out = v;
    return true;
}

bool cmd_json_get_float(const char *json, const char *key, float *out)
{
    char buf[24];
    if (!cmd_json_get(json, key, buf, sizeof(buf))) return false;
    char *end = NULL;
    const float v = strtof(buf, &end);
    if (end == buf || isnan(v) || isinf(v)) return false;
    *out = v;
    return true;
}

void line_assembler_init(LineAssembler *la)
{
    memset(la, 0, sizeof(*la));
}

bool line_assembler_feed(LineAssembler *la, char c)
{
    if (c == '\n' || c == '\r') {
        const bool ready = la->len > 0 && !la->overflow;
        if (la->overflow) la->lines_dropped++;
        if (ready) la->buf[la->len] = '\0';
        la->len = 0;
        la->overflow = false;
        return ready;
    }
    if (la->len < sizeof(la->buf) - 1) {
        la->buf[la->len++] = c;
    } else {
        la->overflow = true;
    }
    return false;
}
