#include "test_common.h"
#include "test_suites.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "csv_writer.h"
#include "svg_plot.h"

void default_simulation(uint64_t seed, kf_sim_result *out) {
  kf_sim_config cfg = kf_sim_config_default();
  cfg.seed = seed;
  cfg.dropouts = kf_default_dropouts(&cfg.dropouts_len);
  size_t route_len = 0;
  const kf_segment *route = kf_default_route(&route_len);
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, route_len, out));
}

kf_sim_config straight_config(void) {
  kf_sim_config cfg = kf_sim_config_default();
  cfg.initial_speed_mps = 10.0;
  cfg.initial_heading_rad = 0.0;
  return cfg;
}

bool *make_fix_flags(const kf_sim_result *sim) {
  bool *flags = calloc(sim->truth_len, sizeof *flags);
  TEST_ASSERT_NOT_NULL(flags);
  for (size_t i = 0; i < sim->gps_len; ++i) {
    if (sim->gps[i].valid) flags[sim->gps[i].step] = true;
  }
  return flags;
}

bool pred_has_fix(const kf_truth_sample *t, void *user) {
  return ((const bool *)user)[t->step];
}

bool pred_in_dropout(const kf_truth_sample *t, void *user) {
  const kf_sim_result *sim = (const kf_sim_result *)user;
  return kf_is_in_windows(t->t, sim->dropouts, sim->dropouts_len);
}

bool pred_before_10s(const kf_truth_sample *t, void *user) {
  (void)user;
  return t->t < 10.0;
}

char *capture_csv(const kf_sim_result *sim, const kf_estimates *est) {
  char *buf = NULL;
  size_t size = 0;
  FILE *fp = open_memstream(&buf, &size);
  if (fp == NULL) return NULL;
  const kf_status st = kf_write_results_csv(fp, sim, est);
  fclose(fp);
  if (st != KF_OK) {
    free(buf);
    return NULL;
  }
  return buf;
}

char *capture_svg(const kf_sim_result *sim, const kf_estimates *est, const kf_svg_options *opt) {
  char *buf = NULL;
  size_t size = 0;
  FILE *fp = open_memstream(&buf, &size);
  if (fp == NULL) return NULL;
  const kf_status st = kf_write_svg(fp, sim, est, opt);
  fclose(fp);
  if (st != KF_OK) {
    free(buf);
    return NULL;
  }
  return buf;
}

size_t count_char(const char *s, char c) {
  size_t n = 0;
  for (; *s != '\0'; ++s)
    if (*s == c) ++n;
  return n;
}

size_t count_substr(const char *haystack, const char *needle) {
  size_t n = 0;
  const size_t len = strlen(needle);
  for (const char *p = strstr(haystack, needle); p != NULL; p = strstr(p + len, needle)) ++n;
  return n;
}

void write_temp_file(const char *content, char *path, size_t path_size) {
  char tmpl[] = "/tmp/kf_test_XXXXXX";
  TEST_ASSERT_TRUE(strlen(tmpl) < path_size);
  const int fd = mkstemp(tmpl);
  TEST_ASSERT_TRUE_MESSAGE(fd >= 0, "mkstemp failed");
  const size_t len = strlen(content);
  const ssize_t written = write(fd, content, len);
  close(fd);
  TEST_ASSERT_TRUE_MESSAGE(written >= 0 && (size_t)written == len, "short write to temp file");
  strcpy(path, tmpl);
}
