#include "data_loader.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mem.h"

#define LINE_BUF_SIZE 256

typedef struct csv_reader {
  FILE *fp;
  char buf[LINE_BUF_SIZE];
  size_t line_no;
  bool first_content_seen;
} csv_reader;

/*
 * Fetch the next line that carries data (skipping blanks, comments and an
 * optional header). Returns 1 with the line in r->buf, 0 at end of file, or
 * -1 on error with *st set.
 */
static int next_data_line(csv_reader *r, kf_status *st) {
  for (;;) {
    if (fgets(r->buf, sizeof r->buf, r->fp) == NULL) {
      if (ferror(r->fp)) {
        *st = KF_ERR_IO;
        return -1;
      }
      return 0;
    }
    r->line_no++;
    if (strchr(r->buf, '\n') == NULL && !feof(r->fp)) { /* line did not fit */
      *st = KF_ERR_PARSE;
      return -1;
    }

    const char *p = r->buf;
    while (isspace((unsigned char)*p)) ++p;
    if (*p == '\0' || *p == '#') continue;

    if (!r->first_content_seen) {
      r->first_content_seen = true;
      if (isalpha((unsigned char)*p)) continue; /* header line */
    }
    return 1;
  }
}

/* Parse exactly `ncols` comma-separated finite numbers from a line. */
static bool parse_row(const char *line, double *vals, size_t ncols) {
  const char *p = line;
  for (size_t i = 0; i < ncols; ++i) {
    while (isspace((unsigned char)*p)) ++p;
    if (*p == '\0') return false;
    char *end = NULL;
    const double v = strtod(p, &end);
    if (end == p || !isfinite(v)) return false;
    vals[i] = v;
    p = end;
    while (isspace((unsigned char)*p) && *p != '\n') ++p;
    if (i + 1 < ncols) {
      if (*p != ',') return false;
      ++p;
    }
  }
  while (isspace((unsigned char)*p)) ++p;
  return *p == '\0';
}

static kf_status open_reader(csv_reader *r, const char *path) {
  memset(r, 0, sizeof *r);
  r->fp = fopen(path, "r");
  return (r->fp == NULL) ? KF_ERR_IO : KF_OK;
}

kf_status kf_load_route_csv(const char *path, kf_segment **out, size_t *out_len, size_t *err_line) {
  if (path == NULL || out == NULL || out_len == NULL) return KF_ERR_INVALID_ARG;
  *out = NULL;
  *out_len = 0;
  if (err_line != NULL) *err_line = 0;

  csv_reader rd;
  KF_TRY(open_reader(&rd, path));

  kf_segment *items = NULL;
  size_t len = 0, cap = 0;
  kf_status st = KF_OK;
  int rc;
  while ((rc = next_data_line(&rd, &st)) > 0) {
    double v[3];
    if (!parse_row(rd.buf, v, 3) || v[0] < 0.0) {
      st = KF_ERR_PARSE;
      break;
    }
    if (len == cap && (st = kf_grow_array((void **)&items, &cap, sizeof *items)) != KF_OK) break;
    items[len].duration_s = v[0];
    items[len].accel_mps2 = v[1];
    items[len].yaw_rate_rad_s = v[2];
    ++len;
  }
  if (rc < 0 || st != KF_OK) {
    if (st == KF_ERR_PARSE && err_line != NULL) *err_line = rd.line_no;
  } else if (len == 0) {
    st = KF_ERR_PARSE; /* a route needs at least one segment (err_line stays 0) */
  }
  fclose(rd.fp);

  if (st != KF_OK) {
    free(items);
    return st;
  }
  *out = items;
  *out_len = len;
  return KF_OK;
}

kf_status kf_load_dropouts_csv(const char *path, kf_time_window **out, size_t *out_len,
                               size_t *err_line) {
  if (path == NULL || out == NULL || out_len == NULL) return KF_ERR_INVALID_ARG;
  *out = NULL;
  *out_len = 0;
  if (err_line != NULL) *err_line = 0;

  csv_reader rd;
  KF_TRY(open_reader(&rd, path));

  kf_time_window *items = NULL;
  size_t len = 0, cap = 0;
  kf_status st = KF_OK;
  int rc;
  while ((rc = next_data_line(&rd, &st)) > 0) {
    double v[2];
    if (!parse_row(rd.buf, v, 2) || v[0] > v[1]) {
      st = KF_ERR_PARSE;
      break;
    }
    if (len == cap && (st = kf_grow_array((void **)&items, &cap, sizeof *items)) != KF_OK) break;
    items[len].start_s = v[0];
    items[len].end_s = v[1];
    ++len;
  }
  if ((rc < 0 || st != KF_OK) && st == KF_ERR_PARSE && err_line != NULL) *err_line = rd.line_no;
  fclose(rd.fp);

  if (st != KF_OK) {
    free(items);
    return st;
  }
  *out = items;
  *out_len = len;
  return KF_OK;
}
