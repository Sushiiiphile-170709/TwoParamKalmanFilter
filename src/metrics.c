#include "metrics.h"

#include <math.h>
#include <string.h>

/* Running sums, so no temporary array of errors is needed. */
typedef struct accumulator {
  size_t count;
  double sum;
  double sum_sq;
  double max;
} accumulator;

static void acc_add(accumulator *a, double e) {
  a->count++;
  a->sum += e;
  a->sum_sq += e * e;
  a->max = (a->max < e) ? e : a->max;
}

static kf_error_stats acc_finish(const accumulator *a) {
  kf_error_stats s;
  memset(&s, 0, sizeof s);
  s.count = a->count;
  if (a->count == 0) return s;
  const double n = (double)a->count;
  s.max = a->max;
  s.mean = a->sum / n;
  s.rmse = sqrt(a->sum_sq / n);
  return s;
}

double kf_distance(kf_point2d a, kf_point2d b) { return hypot(a.x - b.x, a.y - b.y); }

kf_status kf_compute_stats(const double *errors, size_t count, kf_error_stats *out) {
  if (out == NULL || (errors == NULL && count > 0)) return KF_ERR_INVALID_ARG;
  accumulator acc;
  memset(&acc, 0, sizeof acc);
  for (size_t i = 0; i < count; ++i) acc_add(&acc, errors[i]);
  *out = acc_finish(&acc);
  return KF_OK;
}

kf_status kf_gps_error_stats(const kf_sim_result *sim, kf_truth_predicate keep, void *user,
                             kf_error_stats *out) {
  if (sim == NULL || out == NULL) return KF_ERR_INVALID_ARG;
  accumulator acc;
  memset(&acc, 0, sizeof acc);
  for (size_t i = 0; i < sim->gps_len; ++i) {
    const kf_gps_measurement *m = &sim->gps[i];
    if (!m->valid) continue;
    if (m->step >= sim->truth_len) return KF_ERR_OUT_OF_RANGE;
    const kf_truth_sample *truth = &sim->truth[m->step];
    if (keep != NULL && !keep(truth, user)) continue;
    acc_add(&acc, kf_distance(m->pos, truth->pos));
  }
  *out = acc_finish(&acc);
  return KF_OK;
}

kf_status kf_estimate_error_stats(const kf_sim_result *sim, const kf_estimates *estimates,
                                  kf_truth_predicate keep, void *user, kf_error_stats *out) {
  if (sim == NULL || out == NULL) return KF_ERR_INVALID_ARG;
  accumulator acc;
  memset(&acc, 0, sizeof acc);
  const size_t n = (estimates != NULL) ? estimates->len : 0;
  for (size_t i = 0; i < n; ++i) {
    const kf_estimate_sample *e = &estimates->data[i];
    if (e->step >= sim->truth_len) return KF_ERR_OUT_OF_RANGE;
    const kf_truth_sample *truth = &sim->truth[e->step];
    if (keep != NULL && !keep(truth, user)) continue;
    acc_add(&acc, kf_distance(e->pos, truth->pos));
  }
  *out = acc_finish(&acc);
  return KF_OK;
}
