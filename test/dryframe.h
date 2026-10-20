// dryframe.h — minimal host-side assertion framework for dry-run tests.
// Pure C++14 header. Usage:
//   #include "dryframe.h"
//   int main() { CHECK(cond, "msg"); return DF_SUMMARIZE("name"); }
#ifndef DRYFRAME_H
#define DRYFRAME_H

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

namespace dryframe {

inline int& passCount() { static int n = 0; return n; }
inline int& failCount() { static int n = 0; return n; }

inline void report(bool ok, const std::string& msg, const char* file, int line) {
    if (ok) {
        ++passCount();
    } else {
        ++failCount();
        std::printf("FAIL %s:%d: %s\n", file, line, msg.c_str());
    }
}

template <typename T>
inline std::string toStr(const T& v) {
    std::ostringstream os;
    os << v;
    return os.str();
}

inline int summarize(const char* suite) {
    const int p = passCount();
    const int f = failCount();
    std::printf("[%s] PASS: %d FAIL: %d\n", suite, p, f);
    return f == 0 ? 0 : 1;
}

}  // namespace dryframe

#define CHECK(cond, msg) \
    dryframe::report((cond), (msg), __FILE__, __LINE__)

#define CHECK_EQ(a, b)                                                        \
    do {                                                                      \
        const auto& _df_a = (a);                                              \
        const auto& _df_b = (b);                                              \
        dryframe::report(                                                     \
            _df_a == _df_b,                                                   \
            std::string(#a " == " #b " (got ") + dryframe::toStr(_df_a) +     \
                " vs " + dryframe::toStr(_df_b) + ")",                        \
            __FILE__, __LINE__);                                              \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                 \
    do {                                                                      \
        const double _df_a = static_cast<double>(a);                          \
        const double _df_b = static_cast<double>(b);                          \
        dryframe::report(                                                     \
            std::fabs(_df_a - _df_b) <= (tol),                                \
            std::string(#a " ~= " #b " (got ") + dryframe::toStr(_df_a) +     \
                " vs " + dryframe::toStr(_df_b) + ")",                        \
            __FILE__, __LINE__);                                              \
    } while (0)

#define DF_SUMMARIZE(suite) dryframe::summarize(suite)

#endif  // DRYFRAME_H
