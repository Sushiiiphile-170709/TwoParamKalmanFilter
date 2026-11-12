#include "csv_writer.h"

#include <stddef.h>

/* Every fprintf result is checked; a negative value means the stream failed. */
#define OUT(...)                                  \
  do {                                            \
    if (fprintf(fp, __VA_ARGS__) < 0) return KF_ERR_IO; \
  } while (0)

kf_status kf_write_results_csv(FILE *fp, const kf_sim_result *sim, const kf_estimates *estimates) {
  if (fp == NULL || sim == NULL) return KF_ERR_INVALID_ARG;
  if ((sim->truth_len > 0 && sim->truth == NULL) || (sim->gps_len > 0 && sim->gps == NULL))
    return KF_ERR_INVALID_ARG;
  const size_t est_len = (estimates != NULL) ? estimates->len : 0;
  if (est_len > 0 && estimates->data == NULL) return KF_ERR_INVALID_ARG;

  OUT("t,true_x,true_y,true_vx,true_vy,gps_valid,gps_x,gps_y,"
      "est_x,est_y,est_vx,est_vy,est_pos_std,used_gps\n");

  size_t gps_i = 0;
  size_t est_i = 0;
  for (size_t i = 0; i < sim->truth_len; ++i) {
    const kf_truth_sample *t = &sim->truth[i];
    OUT("%.3f,%.3f,%.3f,%.3f,%.3f,", t->t, t->pos.x, t->pos.y, t->vel.x, t->vel.y);

    while (gps_i < sim->gps_len && sim->gps[gps_i].step < t->step) ++gps_i;
    if (gps_i < sim->gps_len && sim->gps[gps_i].step == t->step) {
      const kf_gps_measurement *g = &sim->gps[gps_i];
      OUT("%d,", g->valid ? 1 : 0);
      if (g->valid) {
        OUT("%.3f,%.3f,", g->pos.x, g->pos.y);
      } else {
        OUT(",,");
      }
    } else {
      OUT(",,,");
    }

    while (est_i < est_len && estimates->data[est_i].step < t->step) ++est_i;
    if (est_i < est_len && estimates->data[est_i].step == t->step) {
      const kf_estimate_sample *e = &estimates->data[est_i];
      OUT("%.3f,%.3f,%.3f,%.3f,%.3f,%d", e->pos.x, e->pos.y, e->vel.x, e->vel.y, e->pos_std,
          e->used_gps ? 1 : 0);
    } else {
      OUT(",,,,,");
    }
    OUT("\n");
  }
  return KF_OK;
}

kf_status kf_write_results_csv_file(const char *path, const kf_sim_result *sim,
                                    const kf_estimates *estimates) {
  if (path == NULL) return KF_ERR_INVALID_ARG;
  FILE *fp = fopen(path, "w");
  if (fp == NULL) return KF_ERR_IO;
  kf_status st = kf_write_results_csv(fp, sim, estimates);
  if (fclose(fp) != 0 && st == KF_OK) st = KF_ERR_IO;
  return st;
}
