#ifndef GPS_KALMAN_TYPES_H
#define GPS_KALMAN_TYPES_H

/**
 * @file types.h
 * @brief Plain data types shared by every module.
 *
 * All positions live in a local flat-earth frame, in meters: +x is east and
 * +y is north. Times are in seconds.
 *
 * kf_sim_result and kf_estimates own heap arrays. Zero-initialise them
 * (`= {0}`) before first use and release them with the matching *_free()
 * function; both free functions can safely be called twice.
 */

#include <stdbool.h>
#include <stddef.h>

/** A 2D point or vector (meters or meters per second, depending on use). */
typedef struct kf_point2d {
  double x;
  double y;
} kf_point2d;

/** A closed time interval [start_s, end_s], in seconds. */
typedef struct kf_time_window {
  double start_s;
  double end_s;
} kf_time_window;

/** Ground truth of the simulated car at one simulation step. */
typedef struct kf_truth_sample {
  size_t step;
  double t;
  kf_point2d pos; /**< true position (m)   */
  kf_point2d vel; /**< true velocity (m/s) */
} kf_truth_sample;

/** One GPS fix. When `valid` is false the receiver had no fix (a tunnel). */
typedef struct kf_gps_measurement {
  size_t step; /**< index of the truth sample this fix belongs to */
  double t;
  bool valid;
  kf_point2d pos; /**< noisy measured position (m); unused if !valid */
} kf_gps_measurement;

/** Filter output for one simulation step. */
typedef struct kf_estimate_sample {
  size_t step;
  double t;
  kf_point2d pos; /**< estimated position (m)                 */
  kf_point2d vel; /**< estimated velocity (m/s)               */
  double pos_std; /**< sqrt(Pxx + Pyy), 2D position sigma (m) */
  bool used_gps;  /**< true if a GPS fix was fused at this step */
} kf_estimate_sample;

/** Everything the simulator produces for one run. */
typedef struct kf_sim_result {
  double dt;                /**< simulation step (s)             */
  kf_truth_sample *truth;   /**< one entry per step              */
  size_t truth_len;
  kf_gps_measurement *gps;  /**< one entry per GPS epoch         */
  size_t gps_len;
  kf_time_window *dropouts; /**< intervals with no GPS reception */
  size_t dropouts_len;
} kf_sim_result;

/** A list of filter estimates, one per step once the filter has started. */
typedef struct kf_estimates {
  kf_estimate_sample *data;
  size_t len;
} kf_estimates;

/** Free the arrays of a simulation result and reset it to all zeros. */
void kf_sim_result_free(kf_sim_result *result);

/** Free the array of an estimate list and reset it to all zeros. */
void kf_estimates_free(kf_estimates *estimates);

#endif /* GPS_KALMAN_TYPES_H */
