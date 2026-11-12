#ifndef GPS_KALMAN_TRACKER_H
#define GPS_KALMAN_TRACKER_H

/**
 * @file tracker.h
 * @brief Runs the Kalman filter over a simulated drive.
 *
 * At every simulation step the filter predicts forward by dt. When a valid
 * GPS fix exists for that step it is fused; during a dropout the estimate
 * coasts on the motion model.
 */

#include "kalman_filter.h"
#include "status.h"
#include "types.h"

/**
 * Run the filter over every step of a simulation.
 *
 * The filter starts at the first valid GPS fix, so no estimates exist for the
 * steps before it. `config` may be NULL for the defaults. On success `out`
 * owns its array (release with kf_estimates_free); on error it is all zeros.
 *
 * @return KF_ERR_INVALID_ARG for a NULL argument, a non-positive dt or a
 *         bad filter config; KF_ERR_NO_MEMORY if allocation fails.
 */
kf_status kf_run_tracker(const kf_sim_result *sim, const kf_config *config,
                         kf_estimates *out);

#endif /* GPS_KALMAN_TRACKER_H */
