/* status codes, checked allocation and the free functions of types.h */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "mem.h"
#include "status.h"
#include "test_common.h"
#include "test_suites.h"
#include "types.h"

static void test_every_status_has_its_own_text(void) {
  const kf_status all[] = {KF_OK,           KF_ERR_INVALID_ARG,  KF_ERR_NOT_INITIALIZED,
                           KF_ERR_SINGULAR, KF_ERR_DIM_MISMATCH, KF_ERR_OUT_OF_RANGE,
                           KF_ERR_NO_MEMORY, KF_ERR_IO,          KF_ERR_PARSE};
  const size_t n = sizeof all / sizeof all[0];
  for (size_t i = 0; i < n; ++i) {
    const char *a = kf_status_str(all[i]);
    TEST_ASSERT_NOT_NULL(a);
    TEST_ASSERT_TRUE(strlen(a) > 0);
    for (size_t j = i + 1; j < n; ++j) TEST_ASSERT_TRUE(strcmp(a, kf_status_str(all[j])) != 0);
  }
  TEST_ASSERT_EQUAL_STRING("ok", kf_status_str(KF_OK));
}

static void test_unknown_status_value_is_handled(void) {
  TEST_ASSERT_EQUAL_STRING("unknown status", kf_status_str((kf_status)12345));
}

static kf_status two_step(kf_status first, int *reached_second) {
  KF_TRY(first);
  *reached_second = 1;
  return KF_OK;
}

static void test_try_macro_returns_early_on_error(void) {
  int reached = 0;
  ASSERT_STATUS(KF_ERR_IO, two_step(KF_ERR_IO, &reached));
  TEST_ASSERT_EQUAL_INT(0, reached);
  ASSERT_STATUS(KF_OK, two_step(KF_OK, &reached));
  TEST_ASSERT_EQUAL_INT(1, reached);
}

static void test_alloc_array_gives_zeroed_memory(void) {
  double *p = NULL;
  ASSERT_STATUS(KF_OK, kf_alloc_array((void **)&p, 100, sizeof *p));
  TEST_ASSERT_NOT_NULL(p);
  for (size_t i = 0; i < 100; ++i) ASSERT_EXACT(0.0, p[i]);
  free(p);
}

static void test_alloc_array_zero_count_is_ok_and_null(void) {
  int *p = (int *)0x1;
  ASSERT_STATUS(KF_OK, kf_alloc_array((void **)&p, 0, sizeof *p));
  TEST_ASSERT_NULL(p);
}

static void test_alloc_array_detects_size_overflow(void) {
  void *p = (void *)0x1;
  ASSERT_STATUS(KF_ERR_NO_MEMORY, kf_alloc_array(&p, SIZE_MAX / 2, 16));
  TEST_ASSERT_NULL(p);
}

static void test_alloc_array_rejects_null_out(void) {
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_alloc_array(NULL, 4, 4));
}

static void test_grow_array_doubles_and_keeps_data(void) {
  int *p = NULL;
  size_t cap = 0;
  ASSERT_STATUS(KF_OK, kf_grow_array((void **)&p, &cap, sizeof *p));
  ASSERT_SIZE_EQ(16, cap);
  for (int i = 0; i < 16; ++i) p[i] = i * 3;
  ASSERT_STATUS(KF_OK, kf_grow_array((void **)&p, &cap, sizeof *p));
  ASSERT_SIZE_EQ(32, cap);
  for (int i = 0; i < 16; ++i) TEST_ASSERT_EQUAL_INT(i * 3, p[i]);
  free(p);
}

static void test_grow_array_detects_overflow_and_keeps_old_array(void) {
  int *p = NULL;
  size_t cap = SIZE_MAX / 2 + 1; /* doubling would wrap around */
  int sentinel;
  p = &sentinel;
  ASSERT_STATUS(KF_ERR_NO_MEMORY, kf_grow_array((void **)&p, &cap, sizeof *p));
  TEST_ASSERT_TRUE(p == &sentinel);
  ASSERT_SIZE_EQ(SIZE_MAX / 2 + 1, cap);
}

static void test_free_functions_are_idempotent_and_null_safe(void) {
  kf_sim_result sim = {0};
  default_simulation(1, &sim);
  TEST_ASSERT_NOT_NULL(sim.truth);
  kf_sim_result_free(&sim);
  TEST_ASSERT_NULL(sim.truth);
  TEST_ASSERT_NULL(sim.gps);
  TEST_ASSERT_NULL(sim.dropouts);
  ASSERT_SIZE_EQ(0, sim.truth_len);
  kf_sim_result_free(&sim); /* second call must be harmless */
  kf_sim_result_free(NULL);

  kf_estimates est = {0};
  kf_estimates_free(&est);
  kf_estimates_free(NULL);
}

void run_core_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_every_status_has_its_own_text);
  RUN_TEST(test_unknown_status_value_is_handled);
  RUN_TEST(test_try_macro_returns_early_on_error);
  RUN_TEST(test_alloc_array_gives_zeroed_memory);
  RUN_TEST(test_alloc_array_zero_count_is_ok_and_null);
  RUN_TEST(test_alloc_array_detects_size_overflow);
  RUN_TEST(test_alloc_array_rejects_null_out);
  RUN_TEST(test_grow_array_doubles_and_keeps_data);
  RUN_TEST(test_grow_array_detects_overflow_and_keeps_old_array);
  RUN_TEST(test_free_functions_are_idempotent_and_null_safe);
}
