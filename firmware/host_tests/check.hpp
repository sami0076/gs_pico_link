// A very small test helper. No framework to install, nothing to learn.
//
// Write a test as a plain main() that calls CHECK/CHECK_EQ and finishes with
//   return test_summary();
// A failing check prints the file, the line and both values, then carries on,
// so one run tells you about every failure rather than only the first.
#pragma once

#include <stdio.h>

inline int g_checks_run    = 0;
inline int g_checks_failed = 0;

// Report one result. Called through the macros below, not directly.
inline void check_report(bool ok, const char *expr, const char *file, int line)
{
    g_checks_run++;
    if (!ok) {
        g_checks_failed++;
        printf("FAIL  %s:%d  %s\n", file, line, expr);
    }
}

// Same, but also prints the two numbers, which is what you actually want to
// see when an angle or a step count comes out wrong.
inline void check_report_eq(long got, long want, const char *expr,
                            const char *file, int line)
{
    g_checks_run++;
    if (got != want) {
        g_checks_failed++;
        printf("FAIL  %s:%d  %s\n        got %ld, want %ld\n",
               file, line, expr, got, want);
    }
}

#define CHECK(cond) check_report((cond), #cond, __FILE__, __LINE__)

#define CHECK_EQ(got, want) \
    check_report_eq((long)(got), (long)(want), #got " == " #want, __FILE__, __LINE__)

// Print the tally and return the process exit code: 0 means everything passed.
inline int test_summary()
{
    if (g_checks_failed == 0) {
        printf("PASS  %d checks\n", g_checks_run);
        return 0;
    }
    printf("FAILED  %d of %d checks\n", g_checks_failed, g_checks_run);
    return 1;
}
