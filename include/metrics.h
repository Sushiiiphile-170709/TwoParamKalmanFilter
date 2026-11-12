#ifndef GPS_KALMAN_METRICS_H
#define GPS_KALMAN_METRICS_H

/**
 * @file metrics.h
 * @brief Error statistics used to judge how well the filter performs.
 */

#include <stdbool.h>
#include <stddef.h>

#include "status.h"
#include "types.h"

/** Summary of a set of position errors (meters). */
typedef struct kf_error_stats {
  size_t count;
  double rmse;
  double mean;
  double max;
} kf_error_stats;

/**
 * Optional filter: only truth samples for which it returns true are counted.
 * `user` is passed through untouched (use it instead of a closure).
 */
typedef bool (*kf_truth_predicate)(const kf_truth_sample *truth, void *user);

/** Euclidean distance between two points. */
double kf_distance(kf_point2d a, kf_point2d b);

/**
 * RMSE, mean and max of non-negative error magnitudes. An empty list gives
 * all-zero stats. @return KF_ERR_INVALID_ARG if errors is NULL with count > 0.
 */
kf_status kf_compute_stats(const double *errors, size_t count,
                           kf_error_stats *out);

/** Error of the valid raw GPS fixes against the truth. `keep` may be NULL. */
kf_status kf_gps_error_stats(const kf_sim_result *sim, kf_truth_predicate keep,
                             void *user, kf_error_stats *out);

/**
 * Error of the filter estimates against the truth. `keep` may be NULL.
 * @return KF_ERR_OUT_OF_RANGE if an estimate refers to a missing truth step.
 */
kf_status kf_estimate_error_stats(const kf_sim_result *sim,
                                  const kf_estimates *estimates,
                                  kf_truth_predicate keep, void *user,
                                  kf_error_stats *out);

#endif /* GPS_KALMAN_METRICS_H */
