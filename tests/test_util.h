#ifndef TEST_UTIL_H
#define TEST_UTIL_H

/**
 * @file test_util.h
 *
 * Minimal test helpers for the host tests: checks that record failures and
 * carry on, and a runner macro that reports each test.
 */

#include <stdio.h>
#include <math.h>

extern int test_failures;   // Total failed checks
extern int test_checks;     // Total checks made

#define CHECK(cond) do { \
    test_checks++; \
    if (!(cond)) { \
        test_failures++; \
        printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

// Like CHECK, but prints the values involved when it fails
#define CHECK_NEAR(actual, expected, tol) do { \
    test_checks++; \
    const double a_ = (double)(actual), e_ = (double)(expected), t_ = (double)(tol); \
    if (!(fabs(a_ - e_) <= t_)) { \
        test_failures++; \
        printf("  FAIL %s:%d: %s = %g, expected %g +/- %g\n", __FILE__, __LINE__, #actual, a_, e_, t_); \
    } \
} while (0)

#define CHECK_EQ_U(actual, expected) do { \
    test_checks++; \
    const unsigned long a_ = (unsigned long)(actual), e_ = (unsigned long)(expected); \
    if (a_ != e_) { \
        test_failures++; \
        printf("  FAIL %s:%d: %s = %lu, expected %lu\n", __FILE__, __LINE__, #actual, a_, e_); \
    } \
} while (0)

#define RUN_TEST(fn) do { \
    const int before_ = test_failures; \
    printf("%s\n", #fn); \
    fn(); \
    printf("  %s\n", test_failures == before_ ? "ok" : "FAILED"); \
} while (0)

#endif // TEST_UTIL_H
