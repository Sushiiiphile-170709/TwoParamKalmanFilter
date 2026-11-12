#ifndef GPS_KALMAN_SIMULATOR_H
#define GPS_KALMAN_SIMULATOR_H

/**
 * @file simulator.h
 * @brief Generates a simulated car trip and the noisy GPS data a receiver
 *        would produce for it.
 *
 * No GPS hardware or log is needed. The car follows a route made of segments
 * (accelerate, cruise, turn), so the true path is known exactly. GPS fixes
 * are the true position plus Gaussian noise, and optional "dropout" windows
 * simulate tunnels where the receiver loses its fix.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "status.h"
#include "types.h"

/** Upper bound on simulation steps, so a typo cannot ask for gigabytes. */
#define KF_SIM_MAX_STEPS 5000000UL

/** One piece of the route: constant acceleration and constant yaw rate. */
typedef struct kf_segment {
  double duration_s;
  double accel_mps2;     /**< along-track acceleration (m/s^2)            */
  double yaw_rate_rad_s; /**< turn rate, positive = left (counter-clockwise) */
} kf_segment;

/** Simulator settings. */
typedef struct kf_sim_config {
  double dt_s;                /**< truth integration step (s)            */
  double gps_period_s;        /**< time between GPS fixes (s)            */
  double gps_noise_std_m;     /**< GPS error sigma per axis (m)          */
  double initial_speed_mps;
  double initial_heading_rad; /**< 0 = east, pi/2 = north                */
  kf_point2d initial_position;
  const kf_time_window *dropouts; /**< borrowed; copied into the result  */
  size_t dropouts_len;
  uint64_t seed;              /**< same seed => same data                */
} kf_sim_config;

/** dt 0.1 s, GPS every 1 s, 5 m noise, 5 m/s east, no dropouts, seed 42. */
kf_sim_config kf_sim_config_default(void);

/** A 110 s drive: acceleration, two long bends, braking and a final turn. */
const kf_segment *kf_default_route(size_t *len);

/** Two GPS outages: 45-55 s on a straight and 68-73 s inside the right bend. */
const kf_time_window *kf_default_dropouts(size_t *len);

/** True if t lies inside any of the closed windows. */
bool kf_is_in_windows(double t, const kf_time_window *windows, size_t len);

/**
 * Run the simulation. Deterministic: the same inputs give the same data.
 *
 * On success `out` owns its arrays (release with kf_sim_result_free). On any
 * error `out` is set to all zeros.
 *
 * @return KF_ERR_INVALID_ARG for a non-positive or non-finite step, a GPS
 *         period shorter than the step, negative or non-finite noise, speed
 *         or duration, an empty route, or a run longer than KF_SIM_MAX_STEPS;
 *         KF_ERR_NO_MEMORY if allocation fails.
 */
kf_status kf_simulate(const kf_sim_config *config, const kf_segment *route,
                      size_t route_len, kf_sim_result *out);

#endif /* GPS_KALMAN_SIMULATOR_H */
