#ifndef GPS_KALMAN_MATRIX_H
#define GPS_KALMAN_MATRIX_H

/**
 * @file matrix.h
 * @brief Tiny fixed-capacity dense matrix with the operations a Kalman filter
 *        needs.
 *
 * A kf_mat holds up to 4x4 doubles inline (no heap), which is all the filter
 * ever needs. Shapes are checked at run time: a mismatch returns
 * KF_ERR_DIM_MISMATCH instead of reading out of bounds.
 *
 * Every operation allows the output to be the same object as an input
 * (`kf_mat_mul(&a, &b, &a)` is fine). When an operation fails the output is
 * left untouched.
 */

#include <stddef.h>

#include "status.h"

#define KF_MAT_MAX_DIM 4
#define KF_MAT_DEFAULT_PIVOT_TOL 1e-12

typedef struct kf_mat {
  size_t rows;
  size_t cols;
  double d[KF_MAT_MAX_DIM * KF_MAT_MAX_DIM]; /**< row-major */
} kf_mat;

/**
 * Unchecked element access for a matrix whose shape is already known to be
 * valid, e.g. `KF_MAT_AT(m, 1, 2) = 5.0;`. Use kf_mat_get/kf_mat_set when the
 * indices come from outside.
 */
#define KF_MAT_AT(m, r, c) ((m).d[(r) * (m).cols + (c)])

/** rows x cols matrix of zeros. Both sizes must be in 1..KF_MAT_MAX_DIM. */
kf_status kf_mat_zero(kf_mat *m, size_t rows, size_t cols);

/** n x n identity matrix. */
kf_status kf_mat_identity(kf_mat *m, size_t n);

/** Fill from row-major values; `count` must equal rows * cols. */
kf_status kf_mat_from_values(kf_mat *m, size_t rows, size_t cols,
                             const double *values, size_t count);

/** Bounds-checked read. KF_ERR_OUT_OF_RANGE for a bad index. */
kf_status kf_mat_get(const kf_mat *m, size_t r, size_t c, double *out);

/** Bounds-checked write. KF_ERR_OUT_OF_RANGE for a bad index. */
kf_status kf_mat_set(kf_mat *m, size_t r, size_t c, double value);

kf_status kf_mat_transpose(const kf_mat *a, kf_mat *out);
kf_status kf_mat_add(const kf_mat *a, const kf_mat *b, kf_mat *out);
kf_status kf_mat_sub(const kf_mat *a, const kf_mat *b, kf_mat *out);
kf_status kf_mat_scale(const kf_mat *a, double s, kf_mat *out);

/** Matrix product, (R x K) * (K x C) -> (R x C). */
kf_status kf_mat_mul(const kf_mat *a, const kf_mat *b, kf_mat *out);

/**
 * Invert a square matrix by Gauss-Jordan elimination with partial pivoting.
 * A pivot whose magnitude is below `pivot_tolerance` (or is NaN) counts as
 * singular and returns KF_ERR_SINGULAR with `out` untouched.
 */
kf_status kf_mat_invert(const kf_mat *m, kf_mat *out, double pivot_tolerance);

#endif /* GPS_KALMAN_MATRIX_H */
