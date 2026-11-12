#include <math.h>

#include "kalman_filter.h"
#include "test_common.h"
#include "test_suites.h"

static kf_filter new_filter(void) {
  kf_filter f;
  ASSERT_STATUS(KF_OK, kf_filter_init(&f, NULL));
  return f;
}

static void test_default_config_values(void) {
  const kf_config c = kf_config_default();
  ASSERT_EXACT(1.0, c.process_noise_accel_std);
  ASSERT_EXACT(5.0, c.measurement_noise_std);
  ASSERT_EXACT(25.0, c.initial_position_variance);
  ASSERT_EXACT(100.0, c.initial_velocity_variance);
}

static void test_using_before_start_is_an_error(void) {
  kf_filter f = new_filter();
  TEST_ASSERT_FALSE(kf_filter_is_initialized(&f));
  ASSERT_STATUS(KF_ERR_NOT_INITIALIZED, kf_filter_predict(&f, 1.0));
  ASSERT_STATUS(KF_ERR_NOT_INITIALIZED, kf_filter_update(&f, 0.0, 0.0));
}

static void test_invalid_config_is_rejected(void) {
  kf_filter f;
  kf_config cfg = kf_config_default();
  cfg.measurement_noise_std = 0.0;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_init(&f, &cfg));
  cfg = kf_config_default();
  cfg.process_noise_accel_std = -1.0;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_init(&f, &cfg));
  cfg = kf_config_default();
  cfg.initial_velocity_variance = 0.0;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_init(&f, &cfg));
  cfg = kf_config_default();
  cfg.initial_position_variance = NAN;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_init(&f, &cfg));
  cfg = kf_config_default();
  cfg.measurement_noise_std = INFINITY;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_init(&f, &cfg));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_init(NULL, &cfg));
  cfg = kf_config_default();
  cfg.process_noise_accel_std = 0.0; /* zero process noise is allowed */
  ASSERT_STATUS(KF_OK, kf_filter_init(&f, &cfg));
}

static void test_start_sets_state_and_covariance(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 3.0, 4.0, 1.0, 2.0));
  TEST_ASSERT_TRUE(kf_filter_is_initialized(&f));
  ASSERT_EXACT(3.0, kf_filter_position_x(&f));
  ASSERT_EXACT(4.0, kf_filter_position_y(&f));
  ASSERT_EXACT(1.0, kf_filter_velocity_x(&f));
  ASSERT_EXACT(2.0, kf_filter_velocity_y(&f));
  ASSERT_NEAR(sqrt(5.0), kf_filter_speed(&f), 1e-12);
  ASSERT_EXACT(f.config.initial_position_variance, KF_MAT_AT(*kf_filter_covariance(&f), 0, 0));
  ASSERT_EXACT(f.config.initial_velocity_variance, KF_MAT_AT(*kf_filter_covariance(&f), 2, 2));
}

static void test_start_rejects_non_finite_values(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_start(&f, NAN, 0.0, 0.0, 0.0));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_start(&f, 0.0, INFINITY, 0.0, 0.0));
  TEST_ASSERT_FALSE(kf_filter_is_initialized(&f));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_start(NULL, 0.0, 0.0, 0.0, 0.0));
}

static void test_predict_moves_state_with_velocity(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 10.0, 5.0));
  ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 2.0));
  ASSERT_NEAR(20.0, kf_filter_position_x(&f), 1e-12);
  ASSERT_NEAR(10.0, kf_filter_position_y(&f), 1e-12);
  ASSERT_NEAR(10.0, kf_filter_velocity_x(&f), 1e-12);
  ASSERT_NEAR(5.0, kf_filter_velocity_y(&f), 1e-12);
}

static void test_predict_rejects_bad_dt_and_changes_nothing(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 1.0, 2.0, 3.0, 4.0));
  const kf_filter before = f;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_predict(&f, -0.1));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_predict(&f, NAN));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_predict(&f, INFINITY));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_predict(NULL, 1.0));
  TEST_ASSERT_EQUAL_MEMORY(&before, &f, sizeof f);
  ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 0.0));
}

static void test_predict_grows_uncertainty(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 10.0, 0.0));
  const double before = kf_filter_position_std(&f);
  ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 1.0));
  ASSERT_GT(kf_filter_position_std(&f), before);
}

static void test_prediction_noise_does_not_depend_on_step_size(void) {
  /* One 1 s step and ten 0.1 s steps must inject the same process noise. */
  kf_filter coarse = new_filter(), fine = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&coarse, 0.0, 0.0, 10.0, 0.0));
  ASSERT_STATUS(KF_OK, kf_filter_start(&fine, 0.0, 0.0, 10.0, 0.0));
  ASSERT_STATUS(KF_OK, kf_filter_predict(&coarse, 1.0));
  for (int i = 0; i < 10; ++i) ASSERT_STATUS(KF_OK, kf_filter_predict(&fine, 0.1));
  const kf_mat *pc = kf_filter_covariance(&coarse), *pf = kf_filter_covariance(&fine);
  ASSERT_NEAR(KF_MAT_AT(*pc, 0, 0), KF_MAT_AT(*pf, 0, 0), 1e-9);
  ASSERT_NEAR(KF_MAT_AT(*pc, 2, 2), KF_MAT_AT(*pf, 2, 2), 1e-9);
  ASSERT_NEAR(KF_MAT_AT(*pc, 0, 2), KF_MAT_AT(*pf, 0, 2), 1e-9);
}

static void test_update_shrinks_uncertainty(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 0.0, 0.0));
  ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 1.0));
  const double before = kf_filter_position_std(&f);
  ASSERT_STATUS(KF_OK, kf_filter_update(&f, 0.0, 0.0));
  ASSERT_LT(kf_filter_position_std(&f), before);
}

static void test_update_pulls_estimate_toward_measurement(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 0.0, 0.0));
  ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 1.0));
  ASSERT_STATUS(KF_OK, kf_filter_update(&f, 10.0, -10.0));
  ASSERT_GT(kf_filter_position_x(&f), 0.0);
  ASSERT_LT(kf_filter_position_x(&f), 10.0);
  ASSERT_LT(kf_filter_position_y(&f), 0.0);
  ASSERT_GT(kf_filter_position_y(&f), -10.0);
}

static void test_update_rejects_non_finite_measurement(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 0.0, 0.0));
  const kf_filter before = f;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_update(&f, NAN, 0.0));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_update(&f, 0.0, INFINITY));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_filter_update(NULL, 0.0, 0.0));
  TEST_ASSERT_EQUAL_MEMORY(&before, &f, sizeof f);
}

static void test_singular_innovation_leaves_filter_unchanged(void) {
  /* Tiny variances make S smaller than the inversion tolerance. */
  kf_config cfg = kf_config_default();
  cfg.measurement_noise_std = 1e-7;
  cfg.initial_position_variance = 1e-30;
  kf_filter f;
  ASSERT_STATUS(KF_OK, kf_filter_init(&f, &cfg));
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 1.0, 1.0, 0.0, 0.0));
  const kf_filter before = f;
  ASSERT_STATUS(KF_ERR_SINGULAR, kf_filter_update(&f, 5.0, 5.0));
  TEST_ASSERT_EQUAL_MEMORY(&before, &f, sizeof f);
}

static void test_converges_to_repeated_measurement(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 0.0, 0.0));
  for (int i = 0; i < 50; ++i) {
    ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 1.0));
    ASSERT_STATUS(KF_OK, kf_filter_update(&f, 100.0, 50.0));
  }
  ASSERT_NEAR(100.0, kf_filter_position_x(&f), 0.5);
  ASSERT_NEAR(50.0, kf_filter_position_y(&f), 0.5);
  ASSERT_NEAR(0.0, kf_filter_speed(&f), 0.5);
}

static void test_learns_velocity_from_noise_free_positions(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 0.0, 0.0));
  for (int k = 1; k <= 30; ++k) {
    ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 1.0));
    ASSERT_STATUS(KF_OK, kf_filter_update(&f, 10.0 * k, 3.0 * k)); /* car at (10, 3) m/s */
  }
  ASSERT_NEAR(10.0, kf_filter_velocity_x(&f), 0.2);
  ASSERT_NEAR(3.0, kf_filter_velocity_y(&f), 0.2);
}

static void test_covariance_stays_symmetric_and_positive(void) {
  kf_filter f = new_filter();
  ASSERT_STATUS(KF_OK, kf_filter_start(&f, 0.0, 0.0, 0.0, 0.0));
  for (int k = 1; k <= 100; ++k) {
    ASSERT_STATUS(KF_OK, kf_filter_predict(&f, 0.1));
    if (k % 10 == 0) ASSERT_STATUS(KF_OK, kf_filter_update(&f, k * 1.0, 0.0));
  }
  const kf_mat *p = kf_filter_covariance(&f);
  for (size_t r = 0; r < 4; ++r) {
    ASSERT_GT(KF_MAT_AT(*p, r, r), 0.0);
    for (size_t c = 0; c < 4; ++c) ASSERT_NEAR(KF_MAT_AT(*p, r, c), KF_MAT_AT(*p, c, r), 1e-9);
  }
}

void run_kalman_filter_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_default_config_values);
  RUN_TEST(test_using_before_start_is_an_error);
  RUN_TEST(test_invalid_config_is_rejected);
  RUN_TEST(test_start_sets_state_and_covariance);
  RUN_TEST(test_start_rejects_non_finite_values);
  RUN_TEST(test_predict_moves_state_with_velocity);
  RUN_TEST(test_predict_rejects_bad_dt_and_changes_nothing);
  RUN_TEST(test_predict_grows_uncertainty);
  RUN_TEST(test_prediction_noise_does_not_depend_on_step_size);
  RUN_TEST(test_update_shrinks_uncertainty);
  RUN_TEST(test_update_pulls_estimate_toward_measurement);
  RUN_TEST(test_update_rejects_non_finite_measurement);
  RUN_TEST(test_singular_innovation_leaves_filter_unchanged);
  RUN_TEST(test_converges_to_repeated_measurement);
  RUN_TEST(test_learns_velocity_from_noise_free_positions);
  RUN_TEST(test_covariance_stays_symmetric_and_positive);
}
