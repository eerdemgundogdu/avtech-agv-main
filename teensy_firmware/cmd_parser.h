#ifndef CMD_PARSER_H
#define CMD_PARSER_H

// ============================================================================
//  Jetson -> Teensy komut satiri ayristirici
//
//  Her komut tek satirlik bir JSON nesnesidir (ornek: {"cmd":"manual",
//  "speed":120,"steer":-30}). Tam bir JSON ayristiricisi yerine, bilinen
//  anahtarlari arayan, tahsis yapmayan ve sinirli tamponlarla calisan
//  kucuk bir okuyucu kullanilir.
//
//  Arduino'ya bagimli degil: host'ta firmware_tests/ ile test edilir.
// ============================================================================

#include <stdint.h>
#include <stddef.h>

#define CMD_LINE_MAX 256

// "key" anahtarinin degerini out'a yazar. Tirnakli metinler tirnaksiz
// dondurulur. Anahtar sadece ardindan ':' geliyorsa kabul edilir, yani
// ayni metin bir deger olarak gecerse karismaz. Bulunamazsa, deger bossa
// veya tirnakli metin out'a sigmiyorsa false doner.
bool cmd_json_get(const char *json, const char *key, char *out, size_t max_len);
bool cmd_json_get_long(const char *json, const char *key, long *out);
bool cmd_json_get_float(const char *json, const char *key, float *out);

// Seri porttan gelen baytlari satirlara boler. Tampona sigmayan satir
// tamamen atilir: kesik bir komut asla calistirilmaz.
typedef struct {
    char     buf[CMD_LINE_MAX];
    size_t   len;
    bool     overflow;
    uint32_t lines_dropped;
} LineAssembler;

void line_assembler_init(LineAssembler *la);

// Tam bir satir hazir oldugunda true doner; satir la->buf icindedir
// (NUL ile sonlanmis) ve bir sonraki feed cagrisina kadar gecerlidir.
bool line_assembler_feed(LineAssembler *la, char c);

#endif // CMD_PARSER_H
