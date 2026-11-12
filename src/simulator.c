#include "simulator.h"

#include <math.h>
#include <string.h>

#include "mem.h"
#include "rng.h"

static const kf_segment kDefaultRoute[] = {
    {10.0, 1.0, 0.0},   /* accelerate from 5 to 15 m/s                       */
    {15.0, 0.0, 0.0},   /* cruise straight                                   */
    {15.0, 0.0, 0.1},   /* long left bend (~86 degrees, ~1.5 m/s^2 sideways) */
    {20.0, 0.0, 0.0},   /* straight                                          */
    {15.0, 0.0, -0.1},  /* long right bend (~86 degrees)                     */
    {15.0, 0.0, 0.0},   /* straight                                          */
    {10.0, -1.0, 0.0},  /* brake from 15 to 5 m/s                            */
    {10.0, 0.0, 0.1},   /* final left turn at low speed                      */
};

static const kf_time_window kDefaultDropouts[] = {{45.0, 55.0}, {68.0, 73.0}};

const kf_segment *kf_default_route(size_t *len) {
  if (len != NULL) *len = sizeof kDefaultRoute / sizeof kDefaultRoute[0];
  return kDefaultRoute;
}

const kf_time_window *kf_default_dropouts(size_t *len) {
  if (len != NULL) *len = sizeof kDefaultDropouts / sizeof kDefaultDropouts[0];
  return kDefaultDropouts;
}

kf_sim_config kf_sim_config_default(void) {
  kf_sim_config c;
  memset(&c, 0, sizeof c);
  c.dt_s = 0.1;
  c.gps_period_s = 1.0;
  c.gps_noise_std_m = 5.0;
  c.initial_speed_mps = 5.0;
  c.initial_heading_rad = 0.0;
  c.seed = 42;
  return c;
}

bool kf_is_in_windows(double t, const kf_time_window *windows, size_t len) {
  if (windows == NULL) return false;
  for (size_t i = 0; i < len; ++i) {
    if (t >= windows[i].start_s && t <= windows[i].end_s) return true;
  }
  return false;
}

/* Written so that NaN fails: true only for finite values >= 0. */
static bool finite_nonneg(double v) { return isfinite(v) && v >= 0.0; }

static kf_status validate(const kf_sim_config *cfg, const kf_segment *route, size_t route_len,
                          size_t *total_steps) {
  if (!(cfg->dt_s > 0.0) || !isfinite(cfg->dt_s)) return KF_ERR_INVALID_ARG;
  if (!isfinite(cfg->gps_period_s) || cfg->gps_period_s < cfg->dt_s) return KF_ERR_INVALID_ARG;
  if (!finite_nonneg(cfg->gps_noise_std_m)) return KF_ERR_INVALID_ARG;
  if (!finite_nonneg(cfg->initial_speed_mps)) return KF_ERR_INVALID_ARG;
  if (!isfinite(cfg->initial_heading_rad)) return KF_ERR_INVALID_ARG;
  if (!isfinite(cfg->initial_position.x) || !isfinite(cfg->initial_position.y))
    return KF_ERR_INVALID_ARG;
  if (cfg->dropouts_len > 0 && cfg->dropouts == NULL) return KF_ERR_INVALID_ARG;
  if (route == NULL || route_len == 0) return KF_ERR_INVALID_ARG;

  size_t total = 0;
  for (size_t i = 0; i < route_len; ++i) {
    const kf_segment *s = &route[i];
    if (!finite_nonneg(s->duration_s)) return KF_ERR_INVALID_ARG;
    if (!isfinite(s->accel_mps2) || !isfinite(s->yaw_rate_rad_s)) return KF_ERR_INVALID_ARG;
    const double ratio = s->duration_s / cfg->dt_s;
    if (ratio > (double)KF_SIM_MAX_STEPS) return KF_ERR_INVALID_ARG;
    const size_t steps = (size_t)lround(ratio);
    if (steps > KF_SIM_MAX_STEPS - total) return KF_ERR_INVALID_ARG;
    total += steps;
  }
  *total_steps = total;
  return KF_OK;
}

typedef struct run_state {
  const kf_sim_config *cfg;
  kf_sim_result *out;
  kf_rng rng;
  size_t gps_every;
  size_t gps_next; /* index of the next free slot in out->gps */
  bool noisy;
} run_state;

/* Record the truth at `step`, and a GPS fix if this step is a GPS epoch. */
static void record(run_state *rs, size_t step, kf_point2d pos, double speed, double heading) {
  const double t = (double)step * rs->cfg->dt_s;

  kf_truth_sample *ts = &rs->out->truth[step];
  ts->step = step;
  ts->t = t;
  ts->pos = pos;
  ts->vel.x = speed * cos(heading);
  ts->vel.y = speed * sin(heading);

  if (step % rs->gps_every == 0) {
    kf_gps_measurement *m = &rs->out->gps[rs->gps_next++];
    m->step = step;
    m->t = t;
    m->valid = !kf_is_in_windows(t, rs->cfg->dropouts, rs->cfg->dropouts_len);
    m->pos.x = 0.0;
    m->pos.y = 0.0;
    if (m->valid) {
      /* x is drawn before y; the order matters for reproducing the data. */
      m->pos.x = pos.x + (rs->noisy ? kf_rng_normal(&rs->rng, 0.0, rs->cfg->gps_noise_std_m) : 0.0);
      m->pos.y = pos.y + (rs->noisy ? kf_rng_normal(&rs->rng, 0.0, rs->cfg->gps_noise_std_m) : 0.0);
    }
  }
}

kf_status kf_simulate(const kf_sim_config *config, const kf_segment *route, size_t route_len,
                      kf_sim_result *out) {
  if (out == NULL) return KF_ERR_INVALID_ARG;
  memset(out, 0, sizeof *out);
  if (config == NULL) return KF_ERR_INVALID_ARG;

  size_t total_steps = 0;
  KF_TRY(validate(config, route, route_len, &total_steps));

  /* Number of simulation steps between two GPS fixes (at least 1). */
  double every_d = config->gps_period_s / config->dt_s;
  if (every_d > (double)KF_SIM_MAX_STEPS) every_d = (double)KF_SIM_MAX_STEPS;
  size_t gps_every = (size_t)lround(every_d);
  if (gps_every < 1) gps_every = 1;

  const size_t truth_len = total_steps + 1;
  const size_t gps_len = (truth_len - 1) / gps_every + 1;

  kf_sim_result res;
  memset(&res, 0, sizeof res);
  res.dt = config->dt_s;
  kf_status st = kf_alloc_array((void **)&res.truth, truth_len, sizeof *res.truth);
  if (st == KF_OK) st = kf_alloc_array((void **)&res.gps, gps_len, sizeof *res.gps);
  if (st == KF_OK) st = kf_alloc_array((void **)&res.dropouts, config->dropouts_len, sizeof *res.dropouts);
  if (st != KF_OK) {
    kf_sim_result_free(&res);
    return st;
  }
  res.truth_len = truth_len;
  res.gps_len = gps_len;
  res.dropouts_len = config->dropouts_len;
  if (config->dropouts_len > 0) {
    memcpy(res.dropouts, config->dropouts, config->dropouts_len * sizeof *res.dropouts);
  }

  run_state rs;
  rs.cfg = config;
  rs.out = &res;
  rs.gps_every = gps_every;
  rs.gps_next = 0;
  rs.noisy = config->gps_noise_std_m > 0.0;
  kf_rng_seed(&rs.rng, config->seed);

  kf_point2d pos = config->initial_position;
  double speed = config->initial_speed_mps;
  double heading = config->initial_heading_rad;
  const double dt = config->dt_s;

  size_t step = 0;
  record(&rs, step, pos, speed, heading);
  for (size_t i = 0; i < route_len; ++i) {
    const kf_segment *seg = &route[i];
    const size_t n = (size_t)lround(seg->duration_s / dt);
    for (size_t k = 0; k < n; ++k) {
      const double v = speed + seg->accel_mps2 * dt;
      speed = (0.0 < v) ? v : 0.0; /* a car does not reverse when braking */
      heading += seg->yaw_rate_rad_s * dt;
      pos.x += speed * cos(heading) * dt;
      pos.y += speed * sin(heading) * dt;
      record(&rs, ++step, pos, speed, heading);
    }
  }

  *out = res;
  return KF_OK;
}
