/**
 * @file main.c
 * @brief Command-line demo: simulate a car trip with noisy GPS, run the
 *        Kalman filter over it, print accuracy numbers and write CSV + SVG.
 */

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "csv_writer.h"
#include "data_loader.h"
#include "fsutil.h"
#include "geo.h"
#include "kalman_filter.h"
#include "mem.h"
#include "metrics.h"
#include "simulator.h"
#include "status.h"
#include "svg_plot.h"
#include "tracker.h"

typedef struct options {
  uint64_t seed;
  double gps_noise_m;
  double assumed_noise_m; /* < 0 means "same as gps_noise_m" */
  double accel_std;
  double origin_lat;
  double origin_lon;
  const char *out_dir;
  const char *route_path;    /* NULL = built-in route                         */
  const char *dropouts_path; /* NULL = built-in outages, "none" = no outages  */
  bool help;
} options;

static void print_usage(FILE *to, const char *prog) {
  fprintf(to,
          "Usage: %s [options]\n"
          "  --seed N            RNG seed for the simulated GPS noise (default 42)\n"
          "  --gps-noise M       true GPS noise sigma per axis, meters (default 5)\n"
          "  --assumed-noise M   noise sigma the filter assumes (default: = --gps-noise)\n"
          "  --accel-std A       filter process noise, m/s^2 (default 1.0)\n"
          "  --origin-lat DEG    latitude of the local frame origin (default 45)\n"
          "  --origin-lon DEG    longitude of the local frame origin (default 10)\n"
          "  --route FILE        route segments CSV (default: built-in route)\n"
          "  --dropouts FILE     GPS outage windows CSV, or \"none\" (default: built-in outages)\n"
          "  --out-dir DIR       where to write results.csv and track.svg (default ./output)\n"
          "  --help              show this message\n",
          prog);
}

static bool parse_double(const char *s, double *out) {
  char *end = NULL;
  errno = 0;
  const double v = strtod(s, &end);
  if (end == s || *end != '\0' || errno == ERANGE || !isfinite(v)) return false;
  *out = v;
  return true;
}

static bool parse_u64(const char *s, uint64_t *out) {
  if (!isdigit((unsigned char)s[0])) return false; /* rejects "", "-1", "+1", " 1" */
  char *end = NULL;
  errno = 0;
  const unsigned long long v = strtoull(s, &end, 10);
  if (end == s || *end != '\0' || errno == ERANGE) return false;
  *out = (uint64_t)v;
  return true;
}

/* Returns true on success; on failure writes a message into err. */
static bool parse_args(int argc, char **argv, options *opt, char *err, size_t err_size) {
  opt->seed = 42;
  opt->gps_noise_m = 5.0;
  opt->assumed_noise_m = -1.0;
  opt->accel_std = 1.0;
  opt->origin_lat = 45.0;
  opt->origin_lon = 10.0;
  opt->out_dir = "output";
  opt->route_path = NULL;
  opt->dropouts_path = NULL;
  opt->help = false;

  for (int i = 1; i < argc; ++i) {
    const char *arg = argv[i];
    if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
      opt->help = true;
      continue;
    }

    double *dst_double = NULL;
    const char **dst_str = NULL;
    bool want_seed = false;
    if (strcmp(arg, "--seed") == 0) want_seed = true;
    else if (strcmp(arg, "--gps-noise") == 0) dst_double = &opt->gps_noise_m;
    else if (strcmp(arg, "--assumed-noise") == 0) dst_double = &opt->assumed_noise_m;
    else if (strcmp(arg, "--accel-std") == 0) dst_double = &opt->accel_std;
    else if (strcmp(arg, "--origin-lat") == 0) dst_double = &opt->origin_lat;
    else if (strcmp(arg, "--origin-lon") == 0) dst_double = &opt->origin_lon;
    else if (strcmp(arg, "--out-dir") == 0) dst_str = &opt->out_dir;
    else if (strcmp(arg, "--route") == 0) dst_str = &opt->route_path;
    else if (strcmp(arg, "--dropouts") == 0) dst_str = &opt->dropouts_path;
    else {
      snprintf(err, err_size, "unknown option: %s", arg);
      return false;
    }

    if (i + 1 >= argc) {
      snprintf(err, err_size, "missing value for %s", arg);
      return false;
    }
    const char *value = argv[++i];

    bool ok = true;
    if (want_seed) ok = parse_u64(value, &opt->seed);
    else if (dst_double != NULL) ok = parse_double(value, dst_double);
    else *dst_str = value;
    if (!ok) {
      snprintf(err, err_size, "invalid value for %s: %s", arg, value);
      return false;
    }
  }
  return true;
}

static void print_row(const char *label, const kf_error_stats *s) {
  printf("  %-34s%8.2f%8.2f%8.2f%8zu\n", label, s->rmse, s->mean, s->max, s->count);
}

/* Context for the two "which truth samples count" filters. */
typedef struct report_ctx {
  const kf_sim_result *sim;
  const bool *has_fix; /* one flag per truth step */
} report_ctx;

static bool in_dropout(const kf_truth_sample *t, void *user) {
  const report_ctx *c = (const report_ctx *)user;
  return kf_is_in_windows(t->t, c->sim->dropouts, c->sim->dropouts_len);
}

static bool at_fix(const kf_truth_sample *t, void *user) {
  const report_ctx *c = (const report_ctx *)user;
  return c->has_fix[t->step];
}

static int fail(const char *what, kf_status st) {
  fprintf(stderr, "error: %s: %s\n", what, kf_status_str(st));
  return 1;
}

int main(int argc, char **argv) {
  const char *prog = (argc > 0 && argv[0] != NULL) ? argv[0] : "gps_kalman";

  options opt;
  char errmsg[256];
  if (!parse_args(argc, argv, &opt, errmsg, sizeof errmsg)) {
    fprintf(stderr, "error: %s\n", errmsg);
    print_usage(stderr, prog);
    return 1;
  }
  if (opt.help) {
    print_usage(stdout, prog);
    return 0;
  }

  /* Reject a bad origin up front, before any work is done. */
  kf_local_frame frame;
  const kf_latlon origin = {opt.origin_lat, opt.origin_lon};
  kf_status st = kf_local_frame_init(&frame, origin);
  if (st != KF_OK) {
    fprintf(stderr, "error: bad --origin-lat/--origin-lon: %s (latitude must be in (-90, 90), longitude in [-180, 180])\n",
            kf_status_str(st));
    return 1;
  }

  int exit_code = 1;
  kf_segment *route_owned = NULL;
  kf_time_window *dropouts_owned = NULL;
  bool *has_fix = NULL;
  kf_sim_result sim = {0};
  kf_estimates est = {0};

  /* 0. Route and outages: from files in data/ (or anywhere) or built in. */
  const kf_segment *route = NULL;
  size_t route_len = 0;
  if (opt.route_path != NULL) {
    size_t bad_line = 0;
    st = kf_load_route_csv(opt.route_path, &route_owned, &route_len, &bad_line);
    if (st != KF_OK) {
      fprintf(stderr, "error: cannot load route '%s'", opt.route_path);
      if (bad_line > 0) fprintf(stderr, " (line %zu)", bad_line);
      fprintf(stderr, ": %s\n", kf_status_str(st));
      goto cleanup;
    }
    route = route_owned;
  } else {
    route = kf_default_route(&route_len);
  }

  const kf_time_window *dropouts = NULL;
  size_t dropouts_len = 0;
  if (opt.dropouts_path == NULL) {
    dropouts = kf_default_dropouts(&dropouts_len);
  } else if (strcmp(opt.dropouts_path, "none") != 0) {
    size_t bad_line = 0;
    st = kf_load_dropouts_csv(opt.dropouts_path, &dropouts_owned, &dropouts_len, &bad_line);
    if (st != KF_OK) {
      fprintf(stderr, "error: cannot load dropouts '%s'", opt.dropouts_path);
      if (bad_line > 0) fprintf(stderr, " (line %zu)", bad_line);
      fprintf(stderr, ": %s\n", kf_status_str(st));
      goto cleanup;
    }
    dropouts = dropouts_owned;
  }

  /* 1. Simulate the drive and the noisy GPS receiver. */
  kf_sim_config sim_cfg = kf_sim_config_default();
  sim_cfg.seed = opt.seed;
  sim_cfg.gps_noise_std_m = opt.gps_noise_m;
  sim_cfg.dropouts = dropouts;
  sim_cfg.dropouts_len = dropouts_len;
  if ((st = kf_simulate(&sim_cfg, route, route_len, &sim)) != KF_OK) {
    exit_code = fail("simulation failed", st);
    goto cleanup;
  }

  /* 2. Run the Kalman filter. */
  kf_config kf_cfg = kf_config_default();
  kf_cfg.process_noise_accel_std = opt.accel_std;
  kf_cfg.measurement_noise_std = (opt.assumed_noise_m > 0.0) ? opt.assumed_noise_m : opt.gps_noise_m;
  if (kf_cfg.measurement_noise_std <= 0.0) kf_cfg.measurement_noise_std = 1e-3;
  if ((st = kf_run_tracker(&sim, &kf_cfg, &est)) != KF_OK) {
    exit_code = fail("filter failed", st);
    goto cleanup;
  }

  /* 3. Accuracy report. Flag the steps that have a valid GPS fix, so the
   *    filter and the raw GPS can be compared at the very same instants. */
  if ((st = kf_alloc_array((void **)&has_fix, sim.truth_len, sizeof *has_fix)) != KF_OK) {
    exit_code = fail("report failed", st);
    goto cleanup;
  }
  for (size_t i = 0; i < sim.gps_len; ++i) {
    if (!sim.gps[i].valid) continue;
    if (sim.gps[i].step >= sim.truth_len) {
      exit_code = fail("report failed", KF_ERR_OUT_OF_RANGE);
      goto cleanup;
    }
    has_fix[sim.gps[i].step] = true;
  }
  report_ctx ctx = {&sim, has_fix};

  kf_error_stats raw, at_fixes, outages, all;
  if ((st = kf_gps_error_stats(&sim, NULL, NULL, &raw)) != KF_OK ||
      (st = kf_estimate_error_stats(&sim, &est, at_fix, &ctx, &at_fixes)) != KF_OK ||
      (st = kf_estimate_error_stats(&sim, &est, in_dropout, &ctx, &outages)) != KF_OK ||
      (st = kf_estimate_error_stats(&sim, &est, NULL, NULL, &all)) != KF_OK) {
    exit_code = fail("report failed", st);
    goto cleanup;
  }

  printf("Simulated trip: %.1f s, GPS every %.1f s, noise sigma %.1f m per axis, seed %llu\n\n",
         sim.truth[sim.truth_len - 1].t, sim_cfg.gps_period_s, opt.gps_noise_m,
         (unsigned long long)opt.seed);
  printf("  %-34s%8s%8s%8s%8s\n", "Position error (meters)", "RMSE", "mean", "max", "n");
  print_row("Raw GPS fixes", &raw);
  print_row("Kalman estimate at the same fixes", &at_fixes);
  print_row("Kalman estimate during GPS outages", &outages);
  print_row("Kalman estimate, every step", &all);

  if (est.len > 0) {
    const kf_latlon last = kf_local_frame_to_geodetic(&frame, est.data[est.len - 1].pos);
    printf("\nFinal estimated position: %.6f, %.6f (origin %.6f, %.6f)\n", last.lat_deg, last.lon_deg,
           opt.origin_lat, opt.origin_lon);
  }

  /* 4. Export results. */
  char csv_path[4096], svg_path[4096];
  if ((st = kf_make_dirs(opt.out_dir)) != KF_OK) {
    fprintf(stderr, "error: cannot create output directory '%s': %s\n", opt.out_dir, kf_status_str(st));
    goto cleanup;
  }
  if ((st = kf_path_join(csv_path, sizeof csv_path, opt.out_dir, "results.csv")) != KF_OK ||
      (st = kf_path_join(svg_path, sizeof svg_path, opt.out_dir, "track.svg")) != KF_OK) {
    exit_code = fail("output path is too long", st);
    goto cleanup;
  }
  if ((st = kf_write_results_csv_file(csv_path, &sim, &est)) != KF_OK) {
    fprintf(stderr, "error: cannot write %s: %s\n", csv_path, kf_status_str(st));
    goto cleanup;
  }
  if ((st = kf_write_svg_file(svg_path, &sim, &est, NULL)) != KF_OK) {
    fprintf(stderr, "error: cannot write %s: %s\n", svg_path, kf_status_str(st));
    goto cleanup;
  }
  printf("Wrote %s and %s (open the SVG in a browser)\n", csv_path, svg_path);
  exit_code = 0;

cleanup:
  free(has_fix);
  free(route_owned);
  free(dropouts_owned);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
  return exit_code;
}
