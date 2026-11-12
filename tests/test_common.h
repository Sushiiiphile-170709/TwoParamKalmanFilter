#ifndef KF_TEST_COMMON_H
#define KF_TEST_COMMON_H

/*
 * Helpers shared by the test files: assertion shorthands on top of Unity,
 * canned simulations, truth filters and in-memory output capture.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "metrics.h"
#include "simulator.h"
#include "status.h"
#include "svg_plot.h"
#include "types.h"
#include "unity.h"

/* Check a kf_status; on failure the message names the status that came back. */
#define ASSERT_STATUS(expected, expr)                                          \
  do {                                                                         \
    const kf_status assert_status_ = (expr);                                   \
    TEST_ASSERT_EQUAL_INT_MESSAGE((int)(expected), (int)assert_status_,        \
                                  kf_status_str(assert_status_));              \
  } while (0)

#define ASSERT_SIZE_EQ(expected, actual) \
  TEST_ASSERT_EQUAL_UINT64((uint64_t)(expected), (uint64_t)(actual))

/* Bit-for-bit equality of two doubles (Unity's EQUAL_DOUBLE allows 1e-12). */
#define ASSERT_EXACT(expected, actual) \
  TEST_ASSERT_TRUE_MESSAGE((expected) == (actual), "doubles are not exactly equal")

#define ASSERT_NEAR(expected, actual, delta) \
  TEST_ASSERT_DOUBLE_WITHIN((delta), (expected), (actual))

/* Natural argument order: ASSERT_GT(a, b) means a > b. */
#define ASSERT_GT(actual, threshold) TEST_ASSERT_GREATER_THAN_DOUBLE((threshold), (actual))
#define ASSERT_LT(actual, threshold) TEST_ASSERT_LESS_THAN_DOUBLE((threshold), (actual))

/* Mark which source file the following RUN_TEST lines belong to. */
#define BEGIN_TEST_FILE() (Unity.TestFile = __FILE__)

/* The default route with the default GPS outages. */
void default_simulation(uint64_t seed, kf_sim_result *out);

/* 10 m/s heading east, no outages, no route; the caller supplies the route. */
kf_sim_config straight_config(void);

/* One flag per truth step: true if a valid GPS fix exists at that step. */
bool *make_fix_flags(const kf_sim_result *sim);

/* Truth filters for the metrics functions. */
bool pred_has_fix(const kf_truth_sample *t, void *user);      /* user: const bool *  */
bool pred_in_dropout(const kf_truth_sample *t, void *user);   /* user: const kf_sim_result * */
bool pred_before_10s(const kf_truth_sample *t, void *user);

/* Render CSV / SVG into a malloc'd, NUL-terminated string (NULL on failure). */
char *capture_csv(const kf_sim_result *sim, const kf_estimates *est);
char *capture_svg(const kf_sim_result *sim, const kf_estimates *est, const kf_svg_options *opt);

size_t count_char(const char *s, char c);
size_t count_substr(const char *haystack, const char *needle);

/* Write `content` to a new temporary file; `path` receives its name. */
void write_temp_file(const char *content, char *path, size_t path_size);

#endif /* KF_TEST_COMMON_H */
