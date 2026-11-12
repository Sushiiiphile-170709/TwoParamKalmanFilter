/* The mock-data loaders and the files that ship in data/. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "data_loader.h"
#include "test_common.h"
#include "test_suites.h"

#ifndef KF_TEST_DATA_DIR
#define KF_TEST_DATA_DIR "data"
#endif

static void data_path(char *buf, size_t size, const char *name) {
  snprintf(buf, size, "%s/%s", KF_TEST_DATA_DIR, name);
}

/* Load a route from a temporary file holding `content`. */
static kf_status route_from_text(const char *content, kf_segment **out, size_t *len,
                                 size_t *err_line) {
  char path[64];
  write_temp_file(content, path, sizeof path);
  const kf_status st = kf_load_route_csv(path, out, len, err_line);
  unlink(path);
  return st;
}

static kf_status dropouts_from_text(const char *content, kf_time_window **out, size_t *len,
                                    size_t *err_line) {
  char path[64];
  write_temp_file(content, path, sizeof path);
  const kf_status st = kf_load_dropouts_csv(path, out, len, err_line);
  unlink(path);
  return st;
}

/* ---- the shipped files --------------------------------------------------- */

static void test_shipped_default_route_equals_the_built_in_one(void) {
  char path[512];
  data_path(path, sizeof path, "default_route.csv");
  kf_segment *file = NULL;
  size_t n = 0;
  ASSERT_STATUS(KF_OK, kf_load_route_csv(path, &file, &n, NULL));
  size_t builtin_n = 0;
  const kf_segment *builtin = kf_default_route(&builtin_n);
  ASSERT_SIZE_EQ(builtin_n, n);
  for (size_t i = 0; i < n; ++i) {
    ASSERT_EXACT(builtin[i].duration_s, file[i].duration_s);
    ASSERT_EXACT(builtin[i].accel_mps2, file[i].accel_mps2);
    ASSERT_EXACT(builtin[i].yaw_rate_rad_s, file[i].yaw_rate_rad_s);
  }
  free(file);
}

static void test_shipped_default_dropouts_equal_the_built_in_ones(void) {
  char path[512];
  data_path(path, sizeof path, "default_dropouts.csv");
  kf_time_window *file = NULL;
  size_t n = 0;
  ASSERT_STATUS(KF_OK, kf_load_dropouts_csv(path, &file, &n, NULL));
  size_t builtin_n = 0;
  const kf_time_window *builtin = kf_default_dropouts(&builtin_n);
  ASSERT_SIZE_EQ(builtin_n, n);
  for (size_t i = 0; i < n; ++i) {
    ASSERT_EXACT(builtin[i].start_s, file[i].start_s);
    ASSERT_EXACT(builtin[i].end_s, file[i].end_s);
  }
  free(file);
}

static void test_simulating_from_files_matches_simulating_from_built_ins(void) {
  char rp[512], dp[512];
  data_path(rp, sizeof rp, "default_route.csv");
  data_path(dp, sizeof dp, "default_dropouts.csv");
  kf_segment *route = NULL;
  kf_time_window *drops = NULL;
  size_t rn = 0, dn = 0;
  ASSERT_STATUS(KF_OK, kf_load_route_csv(rp, &route, &rn, NULL));
  ASSERT_STATUS(KF_OK, kf_load_dropouts_csv(dp, &drops, &dn, NULL));

  kf_sim_config cfg = kf_sim_config_default();
  cfg.dropouts = drops;
  cfg.dropouts_len = dn;
  kf_sim_result from_files = {0}, from_code = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, rn, &from_files));
  default_simulation(42, &from_code);

  ASSERT_SIZE_EQ(from_code.truth_len, from_files.truth_len);
  TEST_ASSERT_EQUAL_MEMORY(from_code.truth, from_files.truth, from_code.truth_len * sizeof *from_code.truth);
  TEST_ASSERT_EQUAL_MEMORY(from_code.gps, from_files.gps, from_code.gps_len * sizeof *from_code.gps);
  kf_sim_result_free(&from_files);
  kf_sim_result_free(&from_code);
  free(route);
  free(drops);
}

static void test_shipped_city_files_load_and_simulate(void) {
  char rp[512], dp[512];
  data_path(rp, sizeof rp, "city_route.csv");
  data_path(dp, sizeof dp, "city_dropouts.csv");
  kf_segment *route = NULL;
  kf_time_window *drops = NULL;
  size_t rn = 0, dn = 0;
  ASSERT_STATUS(KF_OK, kf_load_route_csv(rp, &route, &rn, NULL));
  ASSERT_STATUS(KF_OK, kf_load_dropouts_csv(dp, &drops, &dn, NULL));
  ASSERT_SIZE_EQ(10, rn);
  ASSERT_SIZE_EQ(2, dn);
  kf_sim_config cfg = kf_sim_config_default();
  cfg.dropouts = drops;
  cfg.dropouts_len = dn;
  kf_sim_result sim = {0};
  ASSERT_STATUS(KF_OK, kf_simulate(&cfg, route, rn, &sim));
  ASSERT_SIZE_EQ(711, sim.truth_len); /* 71 s */
  kf_sim_result_free(&sim);
  free(route);
  free(drops);
}

/* ---- format handling ----------------------------------------------------- */

static void test_header_is_optional(void) {
  kf_segment *r = NULL;
  size_t n = 0;
  ASSERT_STATUS(KF_OK, route_from_text("10,1,0\n5,0,0.1\n", &r, &n, NULL));
  ASSERT_SIZE_EQ(2, n);
  ASSERT_EXACT(0.1, r[1].yaw_rate_rad_s);
  free(r);
  ASSERT_STATUS(KF_OK, route_from_text("duration_s,accel_mps2,yaw_rate_rad_s\n10,1,0\n", &r, &n, NULL));
  ASSERT_SIZE_EQ(1, n);
  free(r);
}

static void test_comments_blank_lines_whitespace_and_crlf_are_fine(void) {
  kf_segment *r = NULL;
  size_t n = 0;
  const char *text = "# comment\r\n\r\n   \r\nduration_s,accel_mps2,yaw_rate_rad_s\r\n"
                     "  10 , 1.5 , -0.25  \r\n# another\r\n20,0,0";
  ASSERT_STATUS(KF_OK, route_from_text(text, &r, &n, NULL));
  ASSERT_SIZE_EQ(2, n);
  ASSERT_EXACT(10.0, r[0].duration_s);
  ASSERT_EXACT(1.5, r[0].accel_mps2);
  ASSERT_EXACT(-0.25, r[0].yaw_rate_rad_s);
  ASSERT_EXACT(20.0, r[1].duration_s); /* last line without a newline */
  free(r);
}

static void test_more_rows_than_the_first_allocation_are_kept(void) {
  char text[4096] = "";
  for (int i = 0; i < 100; ++i) {
    char row[32];
    snprintf(row, sizeof row, "%d,0,0\n", i + 1);
    strcat(text, row);
  }
  kf_segment *r = NULL;
  size_t n = 0;
  ASSERT_STATUS(KF_OK, route_from_text(text, &r, &n, NULL));
  ASSERT_SIZE_EQ(100, n);
  ASSERT_EXACT(100.0, r[99].duration_s);
  free(r);
}

static void test_bad_rows_report_the_line_number(void) {
  kf_segment *r = (kf_segment *)0x1;
  size_t n = 77, line = 0;
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("# c\nduration_s,a,b\n10,1,0\n5,oops,0\n", &r, &n, &line));
  ASSERT_SIZE_EQ(4, line);
  TEST_ASSERT_NULL(r); /* outputs are reset on failure */
  ASSERT_SIZE_EQ(0, n);
}

static void test_wrong_column_counts_and_bad_values_are_rejected(void) {
  kf_segment *r = NULL;
  size_t n = 0, line = 0;
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("10,1\n", &r, &n, &line));
  ASSERT_SIZE_EQ(1, line);
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("10,1,0,9\n", &r, &n, &line));
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("10;1;0\n", &r, &n, &line));
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("-5,1,0\n", &r, &n, &line));   /* negative duration */
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("10,nan,0\n", &r, &n, &line));
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("10,1,inf\n", &r, &n, &line));
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("10,1,1e999\n", &r, &n, &line));
}

static void test_empty_route_is_an_error_but_empty_dropouts_are_fine(void) {
  kf_segment *r = NULL;
  size_t n = 5, line = 9;
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text("# nothing here\nduration_s,a,b\n", &r, &n, &line));
  ASSERT_SIZE_EQ(0, line); /* a file-level problem, not a line */

  kf_time_window *d = (kf_time_window *)0x1;
  ASSERT_STATUS(KF_OK, dropouts_from_text("# no outages\nstart_s,end_s\n", &d, &n, NULL));
  TEST_ASSERT_NULL(d);
  ASSERT_SIZE_EQ(0, n);
}

static void test_dropout_windows_must_be_ordered(void) {
  kf_time_window *d = NULL;
  size_t n = 0, line = 0;
  ASSERT_STATUS(KF_OK, dropouts_from_text("10,20\n30,30\n", &d, &n, NULL));
  ASSERT_SIZE_EQ(2, n);
  free(d);
  ASSERT_STATUS(KF_ERR_PARSE, dropouts_from_text("start_s,end_s\n10,20\n30,25\n", &d, &n, &line));
  ASSERT_SIZE_EQ(3, line);
  ASSERT_STATUS(KF_ERR_PARSE, dropouts_from_text("1,2,3\n", &d, &n, &line));
}

static void test_very_long_lines_are_rejected(void) {
  char text[600];
  memset(text, '1', sizeof text - 3);
  text[sizeof text - 3] = ',';
  text[sizeof text - 2] = '0';
  text[sizeof text - 1] = '\0';
  kf_segment *r = NULL;
  size_t n = 0, line = 0;
  ASSERT_STATUS(KF_ERR_PARSE, route_from_text(text, &r, &n, &line));
  ASSERT_SIZE_EQ(1, line);
}

static void test_missing_file_and_bad_arguments(void) {
  kf_segment *r = NULL;
  kf_time_window *d = NULL;
  size_t n = 0;
  ASSERT_STATUS(KF_ERR_IO, kf_load_route_csv("/nonexistent/route.csv", &r, &n, NULL));
  ASSERT_STATUS(KF_ERR_IO, kf_load_dropouts_csv("/nonexistent/d.csv", &d, &n, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_load_route_csv(NULL, &r, &n, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_load_route_csv("x", NULL, &n, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_load_route_csv("x", &r, NULL, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_load_dropouts_csv(NULL, &d, &n, NULL));
}

void run_data_loader_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_shipped_default_route_equals_the_built_in_one);
  RUN_TEST(test_shipped_default_dropouts_equal_the_built_in_ones);
  RUN_TEST(test_simulating_from_files_matches_simulating_from_built_ins);
  RUN_TEST(test_shipped_city_files_load_and_simulate);
  RUN_TEST(test_header_is_optional);
  RUN_TEST(test_comments_blank_lines_whitespace_and_crlf_are_fine);
  RUN_TEST(test_more_rows_than_the_first_allocation_are_kept);
  RUN_TEST(test_bad_rows_report_the_line_number);
  RUN_TEST(test_wrong_column_counts_and_bad_values_are_rejected);
  RUN_TEST(test_empty_route_is_an_error_but_empty_dropouts_are_fine);
  RUN_TEST(test_dropout_windows_must_be_ordered);
  RUN_TEST(test_very_long_lines_are_rejected);
  RUN_TEST(test_missing_file_and_bad_arguments);
}
