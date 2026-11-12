#include <math.h>

#include "matrix.h"
#include "test_common.h"
#include "test_suites.h"

static kf_mat make(size_t r, size_t c, const double *v) {
  kf_mat m = {0};
  ASSERT_STATUS(KF_OK, kf_mat_from_values(&m, r, c, v, r * c));
  return m;
}

static void test_zero_matrix_has_shape_and_zeros(void) {
  kf_mat m;
  ASSERT_STATUS(KF_OK, kf_mat_zero(&m, 2, 3));
  ASSERT_SIZE_EQ(2, m.rows);
  ASSERT_SIZE_EQ(3, m.cols);
  for (size_t r = 0; r < 2; ++r)
    for (size_t c = 0; c < 3; ++c) ASSERT_EXACT(0.0, KF_MAT_AT(m, r, c));
}

static void test_invalid_dimensions_are_rejected(void) {
  kf_mat m;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_zero(&m, 0, 2));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_zero(&m, 2, 0));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_zero(&m, KF_MAT_MAX_DIM + 1, 1));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_zero(&m, 1, KF_MAT_MAX_DIM + 1));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_zero(NULL, 2, 2));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_identity(NULL, 2));
  ASSERT_STATUS(KF_OK, kf_mat_zero(&m, KF_MAT_MAX_DIM, KF_MAT_MAX_DIM));
}

static void test_from_values_is_row_major(void) {
  const double v[] = {1, 2, 3, 4};
  const kf_mat m = make(2, 2, v);
  ASSERT_EXACT(1.0, KF_MAT_AT(m, 0, 0));
  ASSERT_EXACT(2.0, KF_MAT_AT(m, 0, 1));
  ASSERT_EXACT(3.0, KF_MAT_AT(m, 1, 0));
  ASSERT_EXACT(4.0, KF_MAT_AT(m, 1, 1));
}

static void test_from_values_with_wrong_count_fails_and_leaves_output(void) {
  const double v[] = {1, 2, 3};
  kf_mat m;
  ASSERT_STATUS(KF_OK, kf_mat_zero(&m, 1, 1));
  KF_MAT_AT(m, 0, 0) = 7.0;
  ASSERT_STATUS(KF_ERR_DIM_MISMATCH, kf_mat_from_values(&m, 2, 2, v, 3));
  ASSERT_SIZE_EQ(1, m.rows);
  ASSERT_EXACT(7.0, KF_MAT_AT(m, 0, 0));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_from_values(&m, 2, 2, NULL, 4));
}

static void test_get_and_set_check_bounds(void) {
  kf_mat m;
  ASSERT_STATUS(KF_OK, kf_mat_zero(&m, 2, 2));
  double v = -1.0;
  ASSERT_STATUS(KF_OK, kf_mat_set(&m, 1, 1, 8.5));
  ASSERT_STATUS(KF_OK, kf_mat_get(&m, 1, 1, &v));
  ASSERT_EXACT(8.5, v);
  ASSERT_STATUS(KF_ERR_OUT_OF_RANGE, kf_mat_get(&m, 2, 0, &v));
  ASSERT_STATUS(KF_ERR_OUT_OF_RANGE, kf_mat_get(&m, 0, 2, &v));
  ASSERT_STATUS(KF_ERR_OUT_OF_RANGE, kf_mat_set(&m, 2, 0, 1.0));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_get(&m, 0, 0, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_get(NULL, 0, 0, &v));
}

static void test_corrupted_shape_is_caught_not_trusted(void) {
  kf_mat bad;
  ASSERT_STATUS(KF_OK, kf_mat_zero(&bad, 2, 2));
  bad.rows = 99; /* fields are public, so someone could do this */
  kf_mat out;
  double v;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_get(&bad, 0, 0, &v));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_transpose(&bad, &out));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_mul(&bad, &bad, &out));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_mat_invert(&bad, &out, 1e-12));
}

static void test_identity_has_ones_on_the_diagonal(void) {
  kf_mat i;
  ASSERT_STATUS(KF_OK, kf_mat_identity(&i, 3));
  for (size_t r = 0; r < 3; ++r)
    for (size_t c = 0; c < 3; ++c) ASSERT_EXACT(r == c ? 1.0 : 0.0, KF_MAT_AT(i, r, c));
}

static void test_product_matches_hand_computation(void) {
  const double av[] = {1, 2, 3, 4, 5, 6};
  const double bv[] = {7, 8, 9, 10, 11, 12};
  const kf_mat a = make(2, 3, av), b = make(3, 2, bv);
  kf_mat c;
  ASSERT_STATUS(KF_OK, kf_mat_mul(&a, &b, &c));
  ASSERT_SIZE_EQ(2, c.rows);
  ASSERT_SIZE_EQ(2, c.cols);
  ASSERT_EXACT(58.0, KF_MAT_AT(c, 0, 0));
  ASSERT_EXACT(64.0, KF_MAT_AT(c, 0, 1));
  ASSERT_EXACT(139.0, KF_MAT_AT(c, 1, 0));
  ASSERT_EXACT(154.0, KF_MAT_AT(c, 1, 1));
}

static void test_transpose_swaps_indices(void) {
  const double av[] = {1, 2, 3, 4, 5, 6};
  const kf_mat a = make(2, 3, av);
  kf_mat t;
  ASSERT_STATUS(KF_OK, kf_mat_transpose(&a, &t));
  ASSERT_SIZE_EQ(3, t.rows);
  ASSERT_SIZE_EQ(2, t.cols);
  ASSERT_EXACT(4.0, KF_MAT_AT(t, 0, 1));
  ASSERT_EXACT(3.0, KF_MAT_AT(t, 2, 0));
  ASSERT_EXACT(6.0, KF_MAT_AT(t, 2, 1));
}

static void test_add_subtract_and_scale(void) {
  const double av[] = {1, 2, 3, 4}, bv[] = {4, 3, 2, 1};
  const kf_mat a = make(2, 2, av), b = make(2, 2, bv);
  kf_mat sum, diff, scaled;
  ASSERT_STATUS(KF_OK, kf_mat_add(&a, &b, &sum));
  ASSERT_STATUS(KF_OK, kf_mat_sub(&a, &b, &diff));
  ASSERT_STATUS(KF_OK, kf_mat_scale(&a, 2.0, &scaled));
  ASSERT_EXACT(5.0, KF_MAT_AT(sum, 0, 0));
  ASSERT_EXACT(5.0, KF_MAT_AT(sum, 1, 1));
  ASSERT_EXACT(-1.0, KF_MAT_AT(diff, 0, 1));
  ASSERT_EXACT(6.0, KF_MAT_AT(scaled, 1, 0));
}

static void test_shape_mismatches_are_reported(void) {
  kf_mat a, b, c, out;
  ASSERT_STATUS(KF_OK, kf_mat_zero(&a, 2, 3));
  ASSERT_STATUS(KF_OK, kf_mat_zero(&b, 2, 2));
  ASSERT_STATUS(KF_OK, kf_mat_zero(&c, 3, 3));
  ASSERT_STATUS(KF_ERR_DIM_MISMATCH, kf_mat_add(&a, &b, &out));
  ASSERT_STATUS(KF_ERR_DIM_MISMATCH, kf_mat_sub(&a, &b, &out));
  ASSERT_STATUS(KF_ERR_DIM_MISMATCH, kf_mat_mul(&a, &b, &out)); /* 2x3 * 2x2 */
  ASSERT_STATUS(KF_ERR_DIM_MISMATCH, kf_mat_invert(&a, &out, 1e-12)); /* not square */
  ASSERT_STATUS(KF_OK, kf_mat_mul(&a, &c, &out));                    /* 2x3 * 3x3 */
}

static void test_operations_work_in_place(void) {
  const double av[] = {1, 2, 3, 4}, bv[] = {0, 1, 1, 0};
  kf_mat a = make(2, 2, av);
  const kf_mat b = make(2, 2, bv);
  ASSERT_STATUS(KF_OK, kf_mat_mul(&a, &b, &a)); /* swaps the columns of a */
  ASSERT_EXACT(2.0, KF_MAT_AT(a, 0, 0));
  ASSERT_EXACT(1.0, KF_MAT_AT(a, 0, 1));
  ASSERT_STATUS(KF_OK, kf_mat_transpose(&a, &a));
  ASSERT_EXACT(4.0, KF_MAT_AT(a, 0, 1));
  ASSERT_STATUS(KF_OK, kf_mat_add(&a, &a, &a));
  ASSERT_EXACT(8.0, KF_MAT_AT(a, 0, 1));
}

static void test_invert_known_2x2(void) {
  const double v[] = {4, 7, 2, 6}; /* det = 10 */
  const kf_mat a = make(2, 2, v);
  kf_mat inv;
  ASSERT_STATUS(KF_OK, kf_mat_invert(&a, &inv, KF_MAT_DEFAULT_PIVOT_TOL));
  ASSERT_NEAR(0.6, KF_MAT_AT(inv, 0, 0), 1e-12);
  ASSERT_NEAR(-0.7, KF_MAT_AT(inv, 0, 1), 1e-12);
  ASSERT_NEAR(-0.2, KF_MAT_AT(inv, 1, 0), 1e-12);
  ASSERT_NEAR(0.4, KF_MAT_AT(inv, 1, 1), 1e-12);
}

static void test_invert_round_trip_4x4(void) {
  const double v[] = {2, 1, 0, 0, 1, 3, 1, 0, 0, 1, 4, 1, 0, 0, 1, 5};
  const kf_mat a = make(4, 4, v);
  kf_mat inv, id;
  ASSERT_STATUS(KF_OK, kf_mat_invert(&a, &inv, KF_MAT_DEFAULT_PIVOT_TOL));
  ASSERT_STATUS(KF_OK, kf_mat_mul(&a, &inv, &id));
  for (size_t r = 0; r < 4; ++r)
    for (size_t c = 0; c < 4; ++c) ASSERT_NEAR(r == c ? 1.0 : 0.0, KF_MAT_AT(id, r, c), 1e-12);
}

static void test_invert_needs_pivoting(void) {
  const double v[] = {0, 1, 1, 0}; /* zero on the diagonal */
  const kf_mat a = make(2, 2, v);
  kf_mat inv;
  ASSERT_STATUS(KF_OK, kf_mat_invert(&a, &inv, KF_MAT_DEFAULT_PIVOT_TOL));
  ASSERT_NEAR(1.0, KF_MAT_AT(inv, 0, 1), 1e-12);
  ASSERT_NEAR(1.0, KF_MAT_AT(inv, 1, 0), 1e-12);
}

static void test_invert_singular_fails_and_leaves_output(void) {
  const double v[] = {1, 2, 2, 4}, nines[] = {9, 9, 9, 9};
  const kf_mat a = make(2, 2, v);
  kf_mat inv = make(2, 2, nines);
  ASSERT_STATUS(KF_ERR_SINGULAR, kf_mat_invert(&a, &inv, KF_MAT_DEFAULT_PIVOT_TOL));
  ASSERT_EXACT(9.0, KF_MAT_AT(inv, 0, 0)); /* output untouched on failure */
}

static void test_invert_treats_nan_as_singular(void) {
  const double v[] = {NAN, 0, 0, 1};
  const kf_mat a = make(2, 2, v);
  kf_mat inv;
  ASSERT_STATUS(KF_ERR_SINGULAR, kf_mat_invert(&a, &inv, KF_MAT_DEFAULT_PIVOT_TOL));
}

void run_matrix_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_zero_matrix_has_shape_and_zeros);
  RUN_TEST(test_invalid_dimensions_are_rejected);
  RUN_TEST(test_from_values_is_row_major);
  RUN_TEST(test_from_values_with_wrong_count_fails_and_leaves_output);
  RUN_TEST(test_get_and_set_check_bounds);
  RUN_TEST(test_corrupted_shape_is_caught_not_trusted);
  RUN_TEST(test_identity_has_ones_on_the_diagonal);
  RUN_TEST(test_product_matches_hand_computation);
  RUN_TEST(test_transpose_swaps_indices);
  RUN_TEST(test_add_subtract_and_scale);
  RUN_TEST(test_shape_mismatches_are_reported);
  RUN_TEST(test_operations_work_in_place);
  RUN_TEST(test_invert_known_2x2);
  RUN_TEST(test_invert_round_trip_4x4);
  RUN_TEST(test_invert_needs_pivoting);
  RUN_TEST(test_invert_singular_fails_and_leaves_output);
  RUN_TEST(test_invert_treats_nan_as_singular);
}
