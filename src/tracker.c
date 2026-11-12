#include "tracker.h"

#include <math.h>
#include <string.h>

#include "mem.h"

kf_status kf_run_tracker(const kf_sim_result *sim, const kf_config *config, kf_estimates *out) {
  if (out == NULL) return KF_ERR_INVALID_ARG;
  memset(out, 0, sizeof *out);
  if (sim == NULL) return KF_ERR_INVALID_ARG;
  if (!(sim->dt > 0.0) || !isfinite(sim->dt)) return KF_ERR_INVALID_ARG;
  if ((sim->truth_len > 0 && sim->truth == NULL) || (sim->gps_len > 0 && sim->gps == NULL))
    return KF_ERR_INVALID_ARG;

  kf_filter filter;
  KF_TRY(kf_filter_init(&filter, config));

  kf_estimates res;
  memset(&res, 0, sizeof res);
  KF_TRY(kf_alloc_array((void **)&res.data, sim->truth_len, sizeof *res.data));

  size_t gps_index = 0;
  for (size_t i = 0; i < sim->truth_len; ++i) {
    const kf_truth_sample *truth = &sim->truth[i];

    /* Find the GPS epoch (if any) that belongs to this step. */
    while (gps_index < sim->gps_len && sim->gps[gps_index].step < truth->step) ++gps_index;
    const kf_gps_measurement *fix = NULL;
    if (gps_index < sim->gps_len && sim->gps[gps_index].step == truth->step) {
      fix = &sim->gps[gps_index];
    }

    /* Predict first (only if the filter is already running), then correct. */
    kf_status st;
    if (kf_filter_is_initialized(&filter) && truth->step > 0) {
      st = kf_filter_predict(&filter, sim->dt);
      if (st != KF_OK) {
        kf_estimates_free(&res);
        return st;
      }
    }

    bool used_gps = false;
    if (fix != NULL && fix->valid) {
      if (!kf_filter_is_initialized(&filter)) {
        st = kf_filter_start(&filter, fix->pos.x, fix->pos.y, 0.0, 0.0);
        used_gps = (st == KF_OK);
      } else {
        st = kf_filter_update(&filter, fix->pos.x, fix->pos.y);
        used_gps = (st == KF_OK);
        if (st == KF_ERR_SINGULAR) st = KF_OK; /* skip this fix, keep going */
      }
      if (st != KF_OK) {
        kf_estimates_free(&res);
        return st;
      }
    }

    if (!kf_filter_is_initialized(&filter)) continue; /* still waiting for the first fix */

    kf_estimate_sample *e = &res.data[res.len++];
    e->step = truth->step;
    e->t = truth->t;
    e->pos.x = kf_filter_position_x(&filter);
    e->pos.y = kf_filter_position_y(&filter);
    e->vel.x = kf_filter_velocity_x(&filter);
    e->vel.y = kf_filter_velocity_y(&filter);
    e->pos_std = kf_filter_position_std(&filter);
    e->used_gps = used_gps;
  }

  *out = res;
  return KF_OK;
}
