/* CSV and SVG writers, plus the small filesystem helpers. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "csv_writer.h"
#include "fsutil.h"
#include "svg_plot.h"
#include "test_common.h"
#include "test_suites.h"
#include "tracker.h"

/* ---- CSV ---------------------------------------------------------------- */

static void test_csv_has_a_header_and_one_row_per_step(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  char *csv = capture_csv(&sim, &est);
  TEST_ASSERT_NOT_NULL(csv);
  TEST_ASSERT_TRUE(strncmp(csv, "t,true_x,true_y,true_vx,true_vy,gps_valid,gps_x,gps_y,", 54) == 0);
  ASSERT_SIZE_EQ(sim.truth_len + 1, count_char(csv, '\n'));
  free(csv);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_csv_rows_always_have_14_columns(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  char *csv = capture_csv(&sim, &est);
  TEST_ASSERT_NOT_NULL(csv);
  for (const char *line = csv; *line != '\0';) {
    const char *end = strchr(line, '\n');
    TEST_ASSERT_NOT_NULL(end);
    size_t commas = 0;
    for (const char *p = line; p < end; ++p)
      if (*p == ',') ++commas;
    ASSERT_SIZE_EQ(13, commas);
    line = end + 1;
  }
  free(csv);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_csv_first_row_and_empty_cells(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  char *csv = capture_csv(&sim, NULL); /* no estimates at all */
  TEST_ASSERT_NOT_NULL(csv);
  const char *row1 = strchr(csv, '\n') + 1;
  TEST_ASSERT_TRUE(strncmp(row1, "0.000,0.000,0.000,5.000,0.000,1,", 32) == 0);
  const char *row1_end = strchr(row1, '\n');
  TEST_ASSERT_TRUE(strncmp(row1_end - 5, ",,,,,", 5) == 0); /* estimate cells empty */
  /* a step without a GPS epoch leaves the three GPS cells empty too */
  const char *row2 = row1_end + 1;
  TEST_ASSERT_TRUE(strncmp(row2, "0.100,", 6) == 0);
  TEST_ASSERT_NOT_NULL(strstr(row2, ",,,,,,,,\n"));
  free(csv);
  kf_sim_result_free(&sim);
}

static void test_csv_marks_dropout_fixes(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  char *csv = capture_csv(&sim, NULL);
  TEST_ASSERT_NOT_NULL(csv);
  ASSERT_SIZE_EQ(17, count_substr(csv, ",0,,,"));
  free(csv);
  kf_sim_result_free(&sim);
}

static void test_csv_rejects_bad_arguments(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_write_results_csv(NULL, &sim, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_write_results_csv(stdout, NULL, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_write_results_csv_file(NULL, &sim, NULL));
  kf_estimates broken = {NULL, 3}; /* length without data */
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_write_results_csv(stdout, &sim, &broken));
  kf_sim_result_free(&sim);
}

static void test_csv_file_round_trip_and_io_error(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  char path[64];
  write_temp_file("", path, sizeof path);
  ASSERT_STATUS(KF_OK, kf_write_results_csv_file(path, &sim, NULL));
  struct stat sb;
  TEST_ASSERT_EQUAL_INT(0, stat(path, &sb));
  TEST_ASSERT_TRUE(sb.st_size > 1000);
  unlink(path);
  ASSERT_STATUS(KF_ERR_IO, kf_write_results_csv_file("/nonexistent_dir/x.csv", &sim, NULL));
  kf_sim_result_free(&sim);
}

/* ---- SVG ---------------------------------------------------------------- */

static void test_svg_is_one_complete_document(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, &(kf_config){1.0, 5.0, 25.0, 100.0}, &est));
  char *svg = capture_svg(&sim, &est, NULL);
  TEST_ASSERT_NOT_NULL(svg);
  TEST_ASSERT_TRUE(strncmp(svg, "<svg ", 5) == 0);
  TEST_ASSERT_TRUE(strcmp(svg + strlen(svg) - 7, "</svg>\n") == 0);
  ASSERT_SIZE_EQ(1, count_substr(svg, "<svg "));
  ASSERT_SIZE_EQ(1, count_substr(svg, "</svg>"));
  ASSERT_SIZE_EQ(1, count_substr(svg, "<g "));
  ASSERT_SIZE_EQ(1, count_substr(svg, "</g>"));
  free(svg);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_svg_draws_truth_estimate_and_each_outage(void) {
  kf_sim_result sim = {0};
  kf_estimates est = {0};
  default_simulation(42, &sim);
  ASSERT_STATUS(KF_OK, kf_run_tracker(&sim, NULL, &est));
  char *svg = capture_svg(&sim, &est, NULL);
  TEST_ASSERT_NOT_NULL(svg);
  /* truth + estimate + one orange stretch for each of the two outages */
  ASSERT_SIZE_EQ(4, count_substr(svg, "<polyline"));
  ASSERT_SIZE_EQ(2, count_substr(svg, "stroke=\"#f08c00\" stroke-width=\"3.0\""));
  free(svg);
  kf_estimates_free(&est);
  kf_sim_result_free(&sim);
}

static void test_svg_gps_dots_can_be_hidden(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  char *with = capture_svg(&sim, NULL, NULL);
  kf_svg_options opt = kf_svg_options_default();
  opt.show_gps = false;
  char *without = capture_svg(&sim, NULL, &opt);
  TEST_ASSERT_NOT_NULL(with);
  TEST_ASSERT_NOT_NULL(without);
  ASSERT_SIZE_EQ(94 + 1, count_substr(with, "<circle")); /* valid fixes + legend dot */
  ASSERT_SIZE_EQ(1, count_substr(without, "<circle"));
  free(with);
  free(without);
  kf_sim_result_free(&sim);
}

static void test_svg_of_empty_data_is_still_valid(void) {
  const kf_sim_result sim = {0};
  char *svg = capture_svg(&sim, NULL, NULL);
  TEST_ASSERT_NOT_NULL(svg);
  ASSERT_SIZE_EQ(0, count_substr(svg, "<polyline"));
  ASSERT_SIZE_EQ(1, count_substr(svg, "</svg>"));
  free(svg);
}

static void test_svg_honors_the_requested_size(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  kf_svg_options opt = kf_svg_options_default();
  opt.width = 400;
  opt.height = 300;
  char *svg = capture_svg(&sim, NULL, &opt);
  TEST_ASSERT_NOT_NULL(svg);
  TEST_ASSERT_NOT_NULL(strstr(svg, "width=\"400\" height=\"300\" viewBox=\"0 0 400 300\""));
  free(svg);
  kf_sim_result_free(&sim);
}

static void test_svg_rejects_bad_arguments(void) {
  kf_sim_result sim = {0};
  default_simulation(42, &sim);
  kf_svg_options bad = kf_svg_options_default();
  bad.width = 0;
  TEST_ASSERT_NULL(capture_svg(&sim, NULL, &bad));
  bad = kf_svg_options_default();
  bad.margin = -5;
  TEST_ASSERT_NULL(capture_svg(&sim, NULL, &bad));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_write_svg(NULL, &sim, NULL, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_write_svg(stdout, NULL, NULL, NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_write_svg_file(NULL, &sim, NULL, NULL));
  ASSERT_STATUS(KF_ERR_IO, kf_write_svg_file("/nonexistent_dir/x.svg", &sim, NULL, NULL));
  kf_sim_result_free(&sim);
}

/* ---- filesystem helpers ------------------------------------------------- */

static void test_make_dirs_creates_nested_directories_and_is_repeatable(void) {
  char base[] = "/tmp/kf_dirs_XXXXXX";
  TEST_ASSERT_NOT_NULL(mkdtemp(base));
  char nested[128];
  snprintf(nested, sizeof nested, "%s/a/b/c", base);
  ASSERT_STATUS(KF_OK, kf_make_dirs(nested));
  ASSERT_STATUS(KF_OK, kf_make_dirs(nested)); /* already there */
  char with_slash[130];
  snprintf(with_slash, sizeof with_slash, "%s/", nested);
  ASSERT_STATUS(KF_OK, kf_make_dirs(with_slash));
  struct stat sb;
  TEST_ASSERT_EQUAL_INT(0, stat(nested, &sb));
  TEST_ASSERT_TRUE(S_ISDIR(sb.st_mode));

  char p[160];
  snprintf(p, sizeof p, "%s/a/b/c", base); rmdir(p);
  snprintf(p, sizeof p, "%s/a/b", base);   rmdir(p);
  snprintf(p, sizeof p, "%s/a", base);     rmdir(p);
  rmdir(base);
}

static void test_make_dirs_fails_on_a_file_and_on_bad_input(void) {
  char file[64];
  write_temp_file("x", file, sizeof file);
  ASSERT_STATUS(KF_ERR_IO, kf_make_dirs(file));
  char below[96];
  snprintf(below, sizeof below, "%s/sub", file);
  ASSERT_STATUS(KF_ERR_IO, kf_make_dirs(below));
  unlink(file);
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_make_dirs(NULL));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_make_dirs(""));
  char huge[5000];
  memset(huge, 'a', sizeof huge - 1);
  huge[sizeof huge - 1] = '\0';
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_make_dirs(huge));
}

static void test_path_join_handles_separators_and_overflow(void) {
  char buf[32];
  ASSERT_STATUS(KF_OK, kf_path_join(buf, sizeof buf, "output", "a.csv"));
  TEST_ASSERT_EQUAL_STRING("output/a.csv", buf);
  ASSERT_STATUS(KF_OK, kf_path_join(buf, sizeof buf, "output/", "a.csv"));
  TEST_ASSERT_EQUAL_STRING("output/a.csv", buf);
  char tiny[8];
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_path_join(tiny, sizeof tiny, "output", "a.csv"));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_path_join(NULL, 8, "a", "b"));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_path_join(buf, 0, "a", "b"));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_path_join(buf, sizeof buf, NULL, "b"));
}

void run_io_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_csv_has_a_header_and_one_row_per_step);
  RUN_TEST(test_csv_rows_always_have_14_columns);
  RUN_TEST(test_csv_first_row_and_empty_cells);
  RUN_TEST(test_csv_marks_dropout_fixes);
  RUN_TEST(test_csv_rejects_bad_arguments);
  RUN_TEST(test_csv_file_round_trip_and_io_error);
  RUN_TEST(test_svg_is_one_complete_document);
  RUN_TEST(test_svg_draws_truth_estimate_and_each_outage);
  RUN_TEST(test_svg_gps_dots_can_be_hidden);
  RUN_TEST(test_svg_of_empty_data_is_still_valid);
  RUN_TEST(test_svg_honors_the_requested_size);
  RUN_TEST(test_svg_rejects_bad_arguments);
  RUN_TEST(test_make_dirs_creates_nested_directories_and_is_repeatable);
  RUN_TEST(test_make_dirs_fails_on_a_file_and_on_bad_input);
  RUN_TEST(test_path_join_handles_separators_and_overflow);
}
