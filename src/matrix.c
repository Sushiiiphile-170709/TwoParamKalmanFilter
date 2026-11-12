#include "matrix.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

static bool dims_ok(size_t rows, size_t cols) {
  return rows >= 1 && rows <= KF_MAT_MAX_DIM && cols >= 1 && cols <= KF_MAT_MAX_DIM;
}

/* The fields are public, so re-check a matrix before trusting its shape. */
static bool mat_ok(const kf_mat *m) {
  return m != NULL && dims_ok(m->rows, m->cols);
}

kf_status kf_mat_zero(kf_mat *m, size_t rows, size_t cols) {
  if (m == NULL || !dims_ok(rows, cols)) return KF_ERR_INVALID_ARG;
  memset(m, 0, sizeof *m);
  m->rows = rows;
  m->cols = cols;
  return KF_OK;
}

kf_status kf_mat_identity(kf_mat *m, size_t n) {
  if (m == NULL) return KF_ERR_INVALID_ARG;
  kf_mat tmp;
  KF_TRY(kf_mat_zero(&tmp, n, n));
  for (size_t i = 0; i < n; ++i) KF_MAT_AT(tmp, i, i) = 1.0;
  *m = tmp;
  return KF_OK;
}

kf_status kf_mat_from_values(kf_mat *m, size_t rows, size_t cols,
                             const double *values, size_t count) {
  if (m == NULL || values == NULL) return KF_ERR_INVALID_ARG;
  kf_mat tmp;
  KF_TRY(kf_mat_zero(&tmp, rows, cols));
  if (count != rows * cols) return KF_ERR_DIM_MISMATCH;
  memcpy(tmp.d, values, count * sizeof(double));
  *m = tmp;
  return KF_OK;
}

kf_status kf_mat_get(const kf_mat *m, size_t r, size_t c, double *out) {
  if (!mat_ok(m) || out == NULL) return KF_ERR_INVALID_ARG;
  if (r >= m->rows || c >= m->cols) return KF_ERR_OUT_OF_RANGE;
  *out = KF_MAT_AT(*m, r, c);
  return KF_OK;
}

kf_status kf_mat_set(kf_mat *m, size_t r, size_t c, double value) {
  if (!mat_ok(m)) return KF_ERR_INVALID_ARG;
  if (r >= m->rows || c >= m->cols) return KF_ERR_OUT_OF_RANGE;
  KF_MAT_AT(*m, r, c) = value;
  return KF_OK;
}

kf_status kf_mat_transpose(const kf_mat *a, kf_mat *out) {
  if (!mat_ok(a) || out == NULL) return KF_ERR_INVALID_ARG;
  kf_mat t;
  KF_TRY(kf_mat_zero(&t, a->cols, a->rows));
  for (size_t r = 0; r < a->rows; ++r)
    for (size_t c = 0; c < a->cols; ++c) KF_MAT_AT(t, c, r) = KF_MAT_AT(*a, r, c);
  *out = t;
  return KF_OK;
}

/* Shared body of add and subtract. */
static kf_status elementwise(const kf_mat *a, const kf_mat *b, kf_mat *out, double sign) {
  if (!mat_ok(a) || !mat_ok(b) || out == NULL) return KF_ERR_INVALID_ARG;
  if (a->rows != b->rows || a->cols != b->cols) return KF_ERR_DIM_MISMATCH;
  kf_mat m;
  KF_TRY(kf_mat_zero(&m, a->rows, a->cols));
  const size_t n = a->rows * a->cols;
  for (size_t i = 0; i < n; ++i) m.d[i] = a->d[i] + sign * b->d[i];
  *out = m;
  return KF_OK;
}

kf_status kf_mat_add(const kf_mat *a, const kf_mat *b, kf_mat *out) {
  return elementwise(a, b, out, 1.0);
}

kf_status kf_mat_sub(const kf_mat *a, const kf_mat *b, kf_mat *out) {
  return elementwise(a, b, out, -1.0);
}

kf_status kf_mat_scale(const kf_mat *a, double s, kf_mat *out) {
  if (!mat_ok(a) || out == NULL) return KF_ERR_INVALID_ARG;
  kf_mat m;
  KF_TRY(kf_mat_zero(&m, a->rows, a->cols));
  const size_t n = a->rows * a->cols;
  for (size_t i = 0; i < n; ++i) m.d[i] = a->d[i] * s;
  *out = m;
  return KF_OK;
}

kf_status kf_mat_mul(const kf_mat *a, const kf_mat *b, kf_mat *out) {
  if (!mat_ok(a) || !mat_ok(b) || out == NULL) return KF_ERR_INVALID_ARG;
  if (a->cols != b->rows) return KF_ERR_DIM_MISMATCH;
  kf_mat m;
  KF_TRY(kf_mat_zero(&m, a->rows, b->cols));
  for (size_t r = 0; r < a->rows; ++r) {
    for (size_t c = 0; c < b->cols; ++c) {
      double sum = 0.0;
      for (size_t k = 0; k < a->cols; ++k) sum += KF_MAT_AT(*a, r, k) * KF_MAT_AT(*b, k, c);
      KF_MAT_AT(m, r, c) = sum;
    }
  }
  *out = m;
  return KF_OK;
}

kf_status kf_mat_invert(const kf_mat *m, kf_mat *out, double pivot_tolerance) {
  if (!mat_ok(m) || out == NULL) return KF_ERR_INVALID_ARG;
  if (m->rows != m->cols) return KF_ERR_DIM_MISMATCH;
  const size_t n = m->rows;

  kf_mat a = *m;
  kf_mat inv;
  KF_TRY(kf_mat_identity(&inv, n));

  for (size_t col = 0; col < n; ++col) {
    size_t pivot = col;
    double best = fabs(KF_MAT_AT(a, col, col));
    for (size_t r = col + 1; r < n; ++r) {
      if (fabs(KF_MAT_AT(a, r, col)) > best) {
        best = fabs(KF_MAT_AT(a, r, col));
        pivot = r;
      }
    }
    if (!(best >= pivot_tolerance)) return KF_ERR_SINGULAR; /* also catches NaN */

    if (pivot != col) {
      for (size_t c = 0; c < n; ++c) {
        const double ta = KF_MAT_AT(a, col, c);
        KF_MAT_AT(a, col, c) = KF_MAT_AT(a, pivot, c);
        KF_MAT_AT(a, pivot, c) = ta;
        const double ti = KF_MAT_AT(inv, col, c);
        KF_MAT_AT(inv, col, c) = KF_MAT_AT(inv, pivot, c);
        KF_MAT_AT(inv, pivot, c) = ti;
      }
    }

    const double diag = KF_MAT_AT(a, col, col);
    for (size_t c = 0; c < n; ++c) {
      KF_MAT_AT(a, col, c) /= diag;
      KF_MAT_AT(inv, col, c) /= diag;
    }

    for (size_t r = 0; r < n; ++r) {
      if (r == col) continue;
      const double factor = KF_MAT_AT(a, r, col);
      if (factor == 0.0) continue;
      for (size_t c = 0; c < n; ++c) {
        KF_MAT_AT(a, r, c) -= factor * KF_MAT_AT(a, col, c);
        KF_MAT_AT(inv, r, c) -= factor * KF_MAT_AT(inv, col, c);
      }
    }
  }

  *out = inv;
  return KF_OK;
}
