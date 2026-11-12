#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "test_common.h"
#include "test_suites.h"
#include "tracker.h"

static void test_filter_beats_raw_gps_at_the_same_fixes(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  bool *flags = make_fix_flags(&sim);
  kf_error_stats raw, filtered;
  ASSERT_STATUS(KF_OK, kf_gps_error_stats(&sim, NULL, NULL, &raw));
  ASSERT_STATUS(KF_OK, kf_estimate_error_stats(&sim, &est, pred_has_fix, flags, &filtered));
  ASSERT_LT(filtered.rmse, raw.rmse); /* on this twisty route: ~6.3 m vs ~7.7 m */
  free(flags);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_one_estimate_per_step_when_gps_starts_at_zero(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  ASSERT_SIZE_EQ(sim.truth_len, est.len);
  for (size_t i = 0; i < est.len; ++i) ASSERT_SIZE_EQ(i, est.data[i].step);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_used_gps_matches_the_valid_fixes(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  bool *flags = make_fix_flags(&sim);
  size_t used = 0;
  for (size_t i = 0; i < est.len; ++i) {
    TEST_ASSERT_EQUAL_INT(flags[est.data[i].step], est.data[i].used_gps);
    if (est.data[i].used_gps) ++used;
  }
  ASSERT_SIZE_EQ(111 - 17, used);
  free(flags);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_filter_waits_for_the_first_valid_fix(void) {
  kf_sim_config cfg = straight_config();
  const kf_time_window windows[] = {{0.0, 4.5}}; /* no GPS for the first 4.5 s */
  cfg.dropouts = windows;
  cfg.dropouts_len = 1;
  const kf_segment route[] = {{20.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  TEST_ASSERT_TRUE(est.len > 0);
  ASSERT_SIZE_EQ(50, est.data[0].step); /* the fix at t = 5 s */
  TEST_ASSERT_TRUE(est.data[0].used_gps);
  ASSERT_SIZE_EQ(sim.truth_len - 50, est.len);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_no_fix_at_all_gives_no_estimates(void) {
  kf_sim_config cfg = straight_config();
  const kf_time_window windows[] = {{0.0, 1000.0}};
  cfg.dropouts = windows;
  cfg.dropouts_len = 1;
  const kf_segment route[] = {{20.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  ASSERT_SIZE_EQ(0, est.len);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static bool after_10s(const kf_truth_sample *t, void *user) {
  (void)user;
  return t->t >= 10.0;
}

static void test_noise_free_straight_drive_is_tracked_closely(void) {
  kf_sim_config cfg = straight_config();
  cfg.gps_noise_std_m = 0.0;
  const kf_segment route[] = {{60.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));

  /* The filter starts with zero velocity, so give it 10 s to learn the speed. */
  kf_error_stats settled;
  ASSERT_STATUS(KF_OK, kf_estimate_error_stats(&sim, &est, after_10s, NULL, &settled));
  ASSERT_SIZE_EQ(501, settled.count);
  ASSERT_LT(settled.max, 0.5);

  const kf_estimate_sample *last = &est.data[est.len - 1];
  ASSERT_NEAR(10.0, last->vel.x, 0.05);
  ASSERT_NEAR(0.0, last->vel.y, 0.05);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static bool pred_has_fix_after_10s(const kf_truth_sample *t, void *user) {
  return t->t >= 10.0 && ((const bool *)user)[t->step];
}

static void test_straight_road_accuracy_matches_the_steady_state_theory(void) {
  kf_sim_config cfg = straight_config();
  const kf_segment route[] = {{300.0, 0.0, 0.0}}; /* constant velocity: the model is exact */
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  bool *flags = make_fix_flags(&sim);
  kf_error_stats raw, filtered;
  ASSERT_STATUS(KF_OK, kf_gps_error_stats(&sim, after_10s, NULL, &raw));
  /* compare at the same instants: steps with a fix, after the 10 s settling time */
  ASSERT_STATUS(KF_OK, kf_estimate_error_stats(&sim, &est, pred_has_fix_after_10s, flags, &filtered));
  ASSERT_SIZE_EQ(raw.count, filtered.count);
  /* Iterating the Riccati recursion for this model (q = 1, R = 25, fix every
   * second) gives a steady-state 2D position error of about 4.84 m, against
   * 7.07 m for the raw fixes. */
  ASSERT_NEAR(4.84, filtered.rmse, 0.8);
  ASSERT_LT(filtered.rmse, raw.rmse * 0.8);
  free(flags);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_uncertainty_grows_in_an_outage_and_shrinks_after(void) {
  kf_sim_config cfg = straight_config();
  const kf_time_window windows[] = {{30.0, 40.0}};
  cfg.dropouts = windows;
  cfg.dropouts_len = 1;
  const kf_segment route[] = {{60.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  const double before = est.data[290].pos_std; /* t = 29 s, just before */
  const double during = est.data[399].pos_std; /* t = 39.9 s, end of the outage */
  const double after = est.data[420].pos_std;  /* t = 42 s, two fixes later */
  ASSERT_GT(during, before * 2.0);
  ASSERT_LT(after, during * 0.5);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_coasting_through_an_outage_beats_freezing(void) {
  kf_sim_config cfg = straight_config();
  const kf_time_window windows[] = {{30.0, 40.0}};
  cfg.dropouts = windows;
  cfg.dropouts_len = 1;
  const kf_segment route[] = {{60.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  /* A frozen estimate would be ~100 m behind after 10 s at 10 m/s. */
  const kf_estimate_sample *e = &est.data[399];
  const double error = kf_distance(e->pos, sim.truth[399].pos);
  ASSERT_LT(error, 25.0);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_lower_process_noise_gives_smoother_velocity_on_a_straight(void) {
  kf_sim_config cfg = straight_config();
  const kf_segment route[] = {{120.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  kf_config calm = kf_config_default(), twitchy = kf_config_default();
  calm.process_noise_accel_std = 0.05;
  twitchy.process_noise_accel_std = 3.0;
  kf_estimates ec = {0}, et = {0};
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, &calm, &ec));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, &twitchy, &et));
  double err_calm = 0.0, err_twitchy = 0.0;
  size_t n = 0;
  for (size_t i = 600; i < ec.len; ++i, ++n) { /* skip the first 60 s of settling */
    err_calm += fabs(ec.data[i].vel.x - 10.0) + fabs(ec.data[i].vel.y);
    err_twitchy += fabs(et.data[i].vel.x - 10.0) + fabs(et.data[i].vel.y);
  }
  ASSERT_GT((double)n, 0.0);
  ASSERT_LT(err_calm, err_twitchy);
  kf_estimates_free(&ec);
  kf_estimates_free(&et);
  kf_sim_result_free(&sim);
}

static void test_tracking_is_deterministic(void) {
  kf_sim_result sim = {0};
  kf_estimates a = {0}, b = {0};
  default_simulation(5, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &a));
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &b));
  ASSERT_SIZE_EQ(a.len, b.len);
  TEST_ASSERT_EQUAL_MEMORY(a.data, b.data, a.len * sizeof *a.data);
  kf_estimates_free(&a);
  kf_estimates_free(&b);
  kf_sim_result_free(&sim);
}

static void test_bad_input_is_rejected_and_output_zeroed(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  kf_estimates est;
  memset(&est, 0xAB, sizeof est);

  kf_config bad = kf_config_default();
  bad.measurement_noise_std = -1.0;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_run_tracker(&sim, &bad, &est));
  TEST_ASSERT_NULL(est.data);
  ASSERT_SIZE_EQ(0, est.len);

  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_run_tracker(NULL, NULL, &est));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_run_tracker(&sim, NULL, NULL));

  const double dt = sim.dt;
  sim.dt = 0.0;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_run_tracker(&sim, NULL, &est));
  sim.dt = dt;

  kf_truth_sample *saved = sim.truth;
  sim.truth = NULL; /* length says there is data, pointer says there is not */
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_run_tracker(&sim, NULL, &est));
  sim.truth = saved;
  kf_sim_result_free(&sim);
}

void run_tracker_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_filter_beats_raw_gps_at_the_same_fixes);
  RUN_TEST(test_one_estimate_per_step_when_gps_starts_at_zero);
  RUN_TEST(test_used_gps_matches_the_valid_fixes);
  RUN_TEST(test_filter_waits_for_the_first_valid_fix);
  RUN_TEST(test_no_fix_at_all_gives_no_estimates);
  RUN_TEST(test_noise_free_straight_drive_is_tracked_closely);
  RUN_TEST(test_straight_road_accuracy_matches_the_steady_state_theory);
  RUN_TEST(test_uncertainty_grows_in_an_outage_and_shrinks_after);
  RUN_TEST(test_coasting_through_an_outage_beats_freezing);
  RUN_TEST(test_lower_process_noise_gives_smoother_velocity_on_a_straight);
  RUN_TEST(test_tracking_is_deterministic);
  RUN_TEST(test_bad_input_is_rejected_and_output_zeroed);
}
