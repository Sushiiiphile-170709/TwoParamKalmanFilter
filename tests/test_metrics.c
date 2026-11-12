#include <math.h>

#include "metrics.h"
#include "test_common.h"
#include "test_suites.h"
#include "tracker.h"

static void test_distance_is_euclidean(void) {
  ASSERT_NEAR(5.0, kf_distance((kf_point2d){0.0, 0.0}, (kf_point2d){3.0, 4.0}), 1e-12);
  ASSERT_EXACT(0.0, kf_distance((kf_point2d){2.0, 2.0}, (kf_point2d){2.0, 2.0}));
}

static void test_stats_of_known_errors(void) {
  const double errors[] = {3.0, 4.0};
  kf_error_stats s;
  ASSERT_STATUS(KF_OK, kf_compute_stats(errors, 2, &s));
  ASSERT_SIZE_EQ(2, s.count);
  ASSERT_NEAR(3.5, s.mean, 1e-12);
  ASSERT_NEAR(sqrt(12.5), s.rmse, 1e-12);
  ASSERT_NEAR(4.0, s.max, 1e-12);
}

static void test_stats_of_nothing_are_zero(void) {
  kf_error_stats s;
  s.count = 99;
  ASSERT_STATUS(KF_OK, kf_compute_stats(NULL, 0, &s));
  ASSERT_SIZE_EQ(0, s.count);
  ASSERT_EXACT(0.0, s.rmse);
  ASSERT_EXACT(0.0, s.mean);
  ASSERT_EXACT(0.0, s.max);
}

static void test_stats_reject_bad_pointers(void) {
  const double e[] = {1.0};
  kf_error_stats s;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_compute_stats(NULL, 1, &s));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_compute_stats(e, 1, NULL));
}

static void test_gps_stats_count_only_valid_fixes(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  kf_error_stats s;
  ASSERT_STATUS(KF_OK, kf_gps_error_stats(&sim, NULL, NULL, &s));
  ASSERT_SIZE_EQ(111 - 17, s.count);
  ASSERT_GT(s.rmse, 0.0);
  kf_sim_result_free(&sim);
}

static void test_gps_stats_honor_the_filter_predicate(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  kf_error_stats s;
  ASSERT_STATUS(KF_OK, kf_gps_error_stats(&sim, pred_before_10s, NULL, &s));
  ASSERT_SIZE_EQ(10, s.count); /* fixes at t = 0, 1, ..., 9 */
  kf_sim_result_free(&sim);
}

static void test_gps_stats_catch_a_fix_pointing_outside_the_truth(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  sim.gps[3].step = sim.truth_len + 10;
  kf_error_stats s;
  ASSERT_STATUS(KF_ERR_OUT_OF_RANGE, kf_gps_error_stats(&sim, NULL, NULL, &s));
  kf_sim_result_free(&sim);
}

static void test_estimate_stats_honor_the_filter_predicate(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  kf_error_stats all, early, dropped;
  ASSERT_STATUS(KF_OK, kf_estimate_error_stats(&sim, &est, NULL, NULL, &all));
  ASSERT_STATUS(KF_OK, kf_estimate_error_stats(&sim, &est, pred_before_10s, NULL, &early));
  ASSERT_STATUS(KF_OK, kf_estimate_error_stats(&sim, &est, pred_in_dropout, &sim, &dropped));
  ASSERT_SIZE_EQ(1101, all.count);
  ASSERT_SIZE_EQ(100, early.count);
  ASSERT_SIZE_EQ(101 + 51, dropped.count); /* 45..55 s and 68..73 s at 0.1 s steps */
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_estimate_stats_with_missing_truth_step_fail(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  kf_estimate_sample bad = {0};
  bad.step = sim.truth_len + 5;
  kf_estimates est = {&bad, 1};
  kf_error_stats s;
  ASSERT_STATUS(KF_ERR_OUT_OF_RANGE, kf_estimate_error_stats(&sim, &est, NULL, NULL, &s));
  kf_sim_result_free(&sim);
}

static void test_null_estimates_count_as_empty(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  kf_error_stats s;
  ASSERT_STATUS(KF_OK, kf_estimate_error_stats(&sim, NULL, NULL, NULL, &s));
  ASSERT_SIZE_EQ(0, s.count);
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_estimate_error_stats(NULL, NULL, NULL, NULL, &s));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_gps_error_stats(&sim, NULL, NULL, NULL));
  kf_sim_result_free(&sim);
}

void run_metrics_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_distance_is_euclidean);
  RUN_TEST(test_stats_of_known_errors);
  RUN_TEST(test_stats_of_nothing_are_zero);
  RUN_TEST(test_stats_reject_bad_pointers);
  RUN_TEST(test_gps_stats_count_only_valid_fixes);
  RUN_TEST(test_gps_stats_honor_the_filter_predicate);
  RUN_TEST(test_gps_stats_catch_a_fix_pointing_outside_the_truth);
  RUN_TEST(test_estimate_stats_honor_the_filter_predicate);
  RUN_TEST(test_estimate_stats_with_missing_truth_step_fail);
  RUN_TEST(test_null_estimates_count_as_empty);
}
