// Bagimliliksiz, tek basliklik kucuk test cercevesi.
#ifndef MINITEST_H
#define MINITEST_H

#include <cstdio>
#include <cstdlib>

namespace minitest {

struct Case {
    const char *name;
    void (*fn)();
    Case *next;
};

inline Case *&head() { static Case *h = nullptr; return h; }
inline int &failures() { static int f = 0; return f; }

struct Registrar {
    Case c;
    Registrar(const char *name, void (*fn)()) : c{name, fn, nullptr} {
        Case **p = &head();
        while (*p) p = &(*p)->next;
        *p = &c;
    }
};

inline int run_all() {
    int n = 0;
    for (Case *c = head(); c; c = c->next) {
        const int before = failures();
        c->fn();
        std::printf("[%s] %s\n", failures() == before ? " OK " : "FAIL", c->name);
        n++;
    }
    std::printf("\n%d test, %d hata\n", n, failures());
    return failures() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace minitest

#define TEST(name)                                                    \
    static void name();                                               \
    static minitest::Registrar name##_reg(#name, &name);              \
    static void name()

#define CHECK(cond)                                                   \
    do {                                                              \
        if (!(cond)) {                                                \
            std::printf("  %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            minitest::failures()++;                                   \
        }                                                             \
    } while (0)

#define CHECK_EQ(a, b)                                                \
    do {                                                              \
        const long long _a = (long long)(a), _b = (long long)(b);     \
        if (_a != _b) {                                               \
            std::printf("  %s:%d: %s == %s  (%lld != %lld)\n",        \
                        __FILE__, __LINE__, #a, #b, _a, _b);          \
            minitest::failures()++;                                   \
        }                                                             \
    } while (0)

#define TEST_MAIN() int main() { return minitest::run_all(); }

#endif
