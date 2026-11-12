#ifndef GPS_KALMAN_KALMAN_FILTER_H
#define GPS_KALMAN_KALMAN_FILTER_H

/**
 * @file kalman_filter.h
 * @brief Linear Kalman filter tracking a car in 2D with a constant-velocity
 *        model.
 *
 * State vector:  x = [px, py, vx, vy]^T   (meters, meters/second)
 * Measurement:   z = [px, py]^T           (GPS position, meters)
 *
 * Motion model, for a time step dt:
 *     px' = px + vx*dt        vx' = vx
 *     py' = py + vy*dt        vy' = vy
 *
 * Real acceleration (braking, turning) is treated as random process noise,
 * using a continuous white-noise-acceleration model for Q. The total noise
 * added over a stretch of time therefore does not depend on how often
 * kf_filter_predict() is called.
 */

#include <stdbool.h>

#include "matrix.h"
#include "status.h"

/** Tuning parameters of the filter. */
typedef struct kf_config {
  /** Unmodelled acceleration: std (m/s^2) of the acceleration change built up
   *  over one second. Bigger = trusts GPS more and reacts faster to turns. */
  double process_noise_accel_std;
  double measurement_noise_std;      /**< assumed GPS sigma per axis (m)   */
  double initial_position_variance;  /**< P0 for position (m^2)            */
  double initial_velocity_variance;  /**< P0 for velocity ((m/s)^2)        */
} kf_config;

/** Defaults: 1.0 m/s^2, 5 m, 25 m^2, 100 (m/s)^2. */
kf_config kf_config_default(void);

/** Filter state. Treat the fields as read-only and use the accessors. */
typedef struct kf_filter {
  kf_config config;
  kf_mat x; /**< 4x1 state      */
  kf_mat p; /**< 4x4 covariance */
  bool initialized;
} kf_filter;

/**
 * Create a filter that has not started yet. `config` may be NULL for the
 * defaults.
 * @return KF_ERR_INVALID_ARG for a non-positive measurement noise or
 *         variance, a negative process noise, or any NaN/infinite value.
 */
kf_status kf_filter_init(kf_filter *filter, const kf_config *config);

/** Start the filter at a state (usually the first GPS fix, zero velocity). */
kf_status kf_filter_start(kf_filter *filter, double px, double py, double vx,
                          double vy);

bool kf_filter_is_initialized(const kf_filter *filter);

/**
 * Propagate state and covariance forward by dt seconds.
 * @return KF_ERR_NOT_INITIALIZED before kf_filter_start();
 *         KF_ERR_INVALID_ARG if dt is negative, NaN or infinite.
 *         The filter is unchanged on any error.
 */
kf_status kf_filter_predict(kf_filter *filter, double dt);

/**
 * Fuse one GPS position.
 * @return KF_ERR_SINGULAR (filter unchanged) if the innovation covariance
 *         cannot be inverted; KF_ERR_NOT_INITIALIZED before start.
 */
kf_status kf_filter_update(kf_filter *filter, double zx, double zy);

double kf_filter_position_x(const kf_filter *filter);
double kf_filter_position_y(const kf_filter *filter);
double kf_filter_velocity_x(const kf_filter *filter);
double kf_filter_velocity_y(const kf_filter *filter);

/** Ground speed estimate (m/s). */
double kf_filter_speed(const kf_filter *filter);

/** 2D position uncertainty, sqrt(Pxx + Pyy) (m). */
double kf_filter_position_std(const kf_filter *filter);

const kf_mat *kf_filter_covariance(const kf_filter *filter);

#endif /* GPS_KALMAN_KALMAN_FILTER_H */
