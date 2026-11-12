#include <math.h>
#include <string.h>

#include "geo.h"
#include "simulator.h"
#include "test_common.h"
#include "test_suites.h"

static void test_default_config_values(void) {
  const kf_sim_config c = kf_sim_config_default();
  ASSERT_EXACT(0.1, c.dt_s);
  ASSERT_EXACT(1.0, c.gps_period_s);
  ASSERT_EXACT(5.0, c.gps_noise_std_m);
  ASSERT_EXACT(5.0, c.initial_speed_mps);
  ASSERT_SIZE_EQ(0, c.dropouts_len);
  TEST_ASSERT_EQUAL_UINT64(42, c.seed);
}

static void test_default_route_and_dropouts(void) {
  size_t n = 0;
  const kf_segment *route = kf_default_route(&n);
  ASSERT_SIZE_EQ(8, n);
  double total = 0.0;
  for (size_t i = 0; i < n; ++i) total += route[i].duration_s;
  ASSERT_NEAR(110.0, total, 1e-12);

  const kf_time_window *w = kf_default_dropouts(&n);
  ASSERT_SIZE_EQ(2, n);
  ASSERT_EXACT(45.0, w[0].start_s);
  ASSERT_EXACT(73.0, w[1].end_s);
  TEST_ASSERT_NOT_NULL(kf_default_route(NULL)); /* length is optional */
}

static void test_window_check_is_inclusive_and_null_safe(void) {
  const kf_time_window w[] = {{10.0, 20.0}, {30.0, 30.0}};
  TEST_ASSERT_TRUE(kf_is_in_windows(10.0, w, 2));
  TEST_ASSERT_TRUE(kf_is_in_windows(20.0, w, 2));
  TEST_ASSERT_TRUE(kf_is_in_windows(30.0, w, 2));
  TEST_ASSERT_FALSE(kf_is_in_windows(9.99, w, 2));
  TEST_ASSERT_FALSE(kf_is_in_windows(25.0, w, 2));
  TEST_ASSERT_FALSE(kf_is_in_windows(15.0, NULL, 0));
}

static void test_step_and_fix_counts_match_the_route(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  ASSERT_SIZE_EQ(1101, sim.truth_len); /* 110 s / 0.1 s, plus the start */
  ASSERT_SIZE_EQ(111, sim.gps_len);    /* a fix every 10 steps, including t = 0 */
  ASSERT_SIZE_EQ(2, sim.dropouts_len);
  ASSERT_EXACT(0.1, sim.dt);
  kf_sim_result_free(&sim);
}

static void test_truth_steps_and_times_are_consistent(void) {
  kf_sim_result sim = {0};
  default_simulation(1, &sim);
  for (size_t i = 0; i < sim.truth_len; ++i) {
    ASSERT_SIZE_EQ(i, sim.truth[i].step);
    ASSERT_NEAR((double)i * 0.1, sim.truth[i].t, 1e-9);
  }
  for (size_t i = 0; i < sim.gps_len; ++i) {
    ASSERT_SIZE_EQ(i * 10, sim.gps[i].step);
    ASSERT_EXACT(sim.truth[sim.gps[i].step].t, sim.gps[i].t);
  }
  kf_sim_result_free(&sim);
}

static void test_straight_segment_covers_expected_distance(void) {
  kf_sim_config cfg = straight_config();
  const kf_segment route[] = {{60.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  ASSERT_NEAR(600.0, sim.truth[sim.truth_len - 1].pos.x, 1e-6);
  ASSERT_NEAR(0.0, sim.truth[sim.truth_len - 1].pos.y, 1e-9);
  kf_sim_result_free(&sim);
}

static void test_acceleration_changes_speed(void) {
  kf_sim_config cfg = kf_sim_config_default(); /* starts at 5 m/s */
  const kf_segment route[] = {{10.0, 1.0, 0.0}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  const kf_truth_sample *last = &sim.truth[sim.truth_len - 1];
  ASSERT_NEAR(15.0, hypot(last->vel.x, last->vel.y), 1e-9);
  kf_sim_result_free(&sim);
}

static void test_braking_never_makes_speed_negative(void) {
  kf_sim_config cfg = kf_sim_config_default();
  const kf_segment route[] = {{20.0, -5.0, 0.0}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  for (size_t i = 0; i < sim.truth_len; ++i) ASSERT_GT(sim.truth[i].vel.x + 1e-12, 0.0);
  ASSERT_NEAR(0.0, sim.truth[sim.truth_len - 1].vel.x, 1e-12);
  kf_sim_result_free(&sim);
}

static void test_positive_yaw_rate_turns_left(void) {
  kf_sim_config cfg = straight_config(); /* heading east */
  const double secs = 1.6;
  const kf_segment route[] = {{secs, 0.0, (KF_PI / 2.0) / secs}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  const kf_truth_sample *last = &sim.truth[sim.truth_len - 1];
  ASSERT_NEAR(0.0, last->vel.x, 1e-9); /* now heading north */
  ASSERT_NEAR(10.0, last->vel.y, 1e-9);
  ASSERT_GT(last->pos.y, 0.0);
  kf_sim_result_free(&sim);
}

static void test_negative_yaw_rate_turns_right(void) {
  kf_sim_config cfg = straight_config();
  const double secs = 1.6;
  const kf_segment route[] = {{secs, 0.0, -(KF_PI / 2.0) / secs}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  const kf_truth_sample *last = &sim.truth[sim.truth_len - 1];
  ASSERT_NEAR(-10.0, last->vel.y, 1e-9); /* now heading south */
  ASSERT_LT(last->pos.y, 0.0);
  kf_sim_result_free(&sim);
}

static void test_zero_duration_segment_is_allowed(void) {
  kf_sim_config cfg = straight_config();
  const kf_segment route[] = {{0.0, 1.0, 1.0}, {1.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 2, &sim));
  ASSERT_SIZE_EQ(11, sim.truth_len);
  kf_sim_result_free(&sim);
}

static void test_zero_noise_gps_equals_truth_exactly(void) {
  kf_sim_config cfg = straight_config();
  cfg.gps_noise_std_m = 0.0;
  const kf_segment route[] = {{30.0, 0.0, 0.05}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  for (size_t i = 0; i < sim.gps_len; ++i) {
    const kf_truth_sample *t = &sim.truth[sim.gps[i].step];
    ASSERT_EXACT(t->pos.x, sim.gps[i].pos.x);
    ASSERT_EXACT(t->pos.y, sim.gps[i].pos.y);
  }
  kf_sim_result_free(&sim);
}

static void test_gps_noise_has_requested_magnitude(void) {
  kf_sim_config cfg = straight_config();
  cfg.gps_noise_std_m = 5.0;
  const kf_segment route[] = {{3000.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  kf_error_stats s;
  ASSERT_STATUS(KF_OK, kf_gps_error_stats(&sim, NULL, NULL, &s));
  ASSERT_SIZE_EQ(3001, s.count);
  ASSERT_NEAR(5.0 * sqrt(2.0), s.rmse, 0.3); /* sigma per axis -> 2D RMSE */
  kf_sim_result_free(&sim);
}

static void test_same_seed_gives_identical_data_and_other_seed_differs(void) {
  kf_sim_result a = {0}, b = {0}, c = {0};
  default_simulation(7, &a);
  default_simulation(7, &b);
  default_simulation(8, &c);
  ASSERT_SIZE_EQ(a.gps_len, b.gps_len);
  TEST_ASSERT_EQUAL_MEMORY(a.gps, b.gps, a.gps_len * sizeof *a.gps);
  TEST_ASSERT_EQUAL_MEMORY(a.truth, b.truth, a.truth_len * sizeof *a.truth);
  TEST_ASSERT_TRUE(a.gps[1].pos.x != c.gps[1].pos.x);
  kf_sim_result_free(&a);
  kf_sim_result_free(&b);
  kf_sim_result_free(&c);
}

static void test_dropout_windows_mark_fixes_invalid(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  size_t invalid = 0;
  for (size_t i = 0; i < sim.gps_len; ++i) {
    const bool inside = kf_is_in_windows(sim.gps[i].t, sim.dropouts, sim.dropouts_len);
    TEST_ASSERT_EQUAL_INT(!inside, sim.gps[i].valid);
    if (!sim.gps[i].valid) ++invalid;
  }
  ASSERT_SIZE_EQ(11 + 6, invalid); /* 45..55 s and 68..73 s, both ends included */
  kf_sim_result_free(&sim);
}

static void test_result_keeps_its_own_copy_of_the_dropouts(void) {
  kf_sim_config cfg = straight_config();
  kf_time_window windows[] = {{1.0, 2.0}};
  cfg.dropouts = windows;
  cfg.dropouts_len = 1;
  const kf_segment route[] = {{5.0, 0.0, 0.0}};
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, 1, &sim));
  windows[0].start_s = 99.0; /* must not affect the result */
  ASSERT_EXACT(1.0, sim.dropouts[0].start_s);
  kf_sim_result_free(&sim);
}

static void test_invalid_settings_are_rejected_and_output_is_zeroed(void) {
  const kf_segment route[] = {{5.0, 0.0, 0.0}};
  kf_sim_result sim;
  kf_sim_config cfg;

#define EXPECT_INVALID(mutation)                                      \
  do {                                                                \
    cfg = straight_config();                                          \
    mutation;                                                         \
    memset(&sim, 0xAB, sizeof sim);                                   \
    ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, route, 1, &sim)); \
    TEST_ASSERT_NULL(sim.truth);                                      \
    ASSERT_SIZE_EQ(0, sim.truth_len);                                 \
  } while (0)

  EXPECT_INVALID(cfg.dt_s = 0.0);
  EXPECT_INVALID(cfg.dt_s = -0.1);
  EXPECT_INVALID(cfg.dt_s = NAN);
  EXPECT_INVALID(cfg.gps_period_s = 0.05); /* shorter than dt */
  EXPECT_INVALID(cfg.gps_period_s = INFINITY);
  EXPECT_INVALID(cfg.gps_noise_std_m = -1.0);
  EXPECT_INVALID(cfg.gps_noise_std_m = NAN);
  EXPECT_INVALID(cfg.initial_speed_mps = -1.0);
  EXPECT_INVALID(cfg.initial_heading_rad = INFINITY);
  EXPECT_INVALID(cfg.initial_position.x = NAN);
  EXPECT_INVALID(cfg.dropouts_len = 3); /* length without an array */
#undef EXPECT_INVALID
}

static void test_invalid_routes_are_rejected(void) {
  const kf_sim_config cfg = straight_config();
  kf_sim_result sim = {0};
  const kf_segment ok[] = {{5.0, 0.0, 0.0}};
  const kf_segment negative[] = {{-1.0, 0.0, 0.0}};
  const kf_segment nan_dur[] = {{NAN, 0.0, 0.0}};
  const kf_segment nan_acc[] = {{1.0, NAN, 0.0}};
  const kf_segment inf_yaw[] = {{1.0, 0.0, INFINITY}};
  const kf_segment huge[] = {{1e12, 0.0, 0.0}};
  const kf_segment two_huge[] = {{400000.0, 0.0, 0.0}, {400000.0, 0.0, 0.0}}; /* 8e6 steps */

  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, NULL, 1, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, ok, 0, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, negative, 1, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, nan_dur, 1, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, nan_acc, 1, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, inf_yaw, 1, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, huge, 1, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, two_huge, 2, &sim)); /* sum too big */
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(NULL, ok, 1, &sim));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_simulate(&cfg, ok, 1, NULL));
}

void run_simulator_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_default_config_values);
  RUN_TEST(test_default_route_and_dropouts);
  RUN_TEST(test_window_check_is_inclusive_and_null_safe);
  RUN_TEST(test_step_and_fix_counts_match_the_route);
  RUN_TEST(test_truth_steps_and_times_are_consistent);
  RUN_TEST(test_straight_segment_covers_expected_distance);
  RUN_TEST(test_acceleration_changes_speed);
  RUN_TEST(test_braking_never_makes_speed_negative);
  RUN_TEST(test_positive_yaw_rate_turns_left);
  RUN_TEST(test_negative_yaw_rate_turns_right);
  RUN_TEST(test_zero_duration_segment_is_allowed);
  RUN_TEST(test_zero_noise_gps_equals_truth_exactly);
  RUN_TEST(test_gps_noise_has_requested_magnitude);
  RUN_TEST(test_same_seed_gives_identical_data_and_other_seed_differs);
  RUN_TEST(test_dropout_windows_mark_fixes_invalid);
  RUN_TEST(test_result_keeps_its_own_copy_of_the_dropouts);
  RUN_TEST(test_invalid_settings_are_rejected_and_output_is_zeroed);
  RUN_TEST(test_invalid_routes_are_rejected);
}
