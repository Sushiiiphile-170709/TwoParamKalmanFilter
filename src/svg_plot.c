#include "svg_plot.h"

#include <math.h>
#include <stddef.h>

#include "simulator.h"

#define OUT(...)                                  \
  do {                                            \
    if (fprintf(fp, __VA_ARGS__) < 0) return KF_ERR_IO; \
  } while (0)

kf_svg_options kf_svg_options_default(void) {
  kf_svg_options o;
  o.width = 900;
  o.height = 700;
  o.margin = 50;
  o.show_gps = true;
  return o;
}

/* Maps local meters to SVG pixels: uniform scale, y axis flipped. */
typedef struct viewport {
  double min_x, min_y, scale, off_x, off_y;
  int height;
} viewport;

static double vp_x(const viewport *v, double x) { return v->off_x + (x - v->min_x) * v->scale; }
static double vp_y(const viewport *v, double y) {
  return (double)v->height - (v->off_y + (y - v->min_y) * v->scale);
}

typedef struct bounds {
  double min_x, min_y, max_x, max_y;
} bounds;

static void extend(bounds *b, kf_point2d p) {
  b->min_x = (p.x < b->min_x) ? p.x : b->min_x;
  b->max_x = (p.x > b->max_x) ? p.x : b->max_x;
  b->min_y = (p.y < b->min_y) ? p.y : b->min_y;
  b->max_y = (p.y > b->max_y) ? p.y : b->max_y;
}

static viewport make_viewport(const kf_sim_result *sim, const kf_estimates *est,
                              const kf_svg_options *opt) {
  bounds b = {INFINITY, INFINITY, -INFINITY, -INFINITY};
  for (size_t i = 0; i < sim->truth_len; ++i) extend(&b, sim->truth[i].pos);
  for (size_t i = 0; i < sim->gps_len; ++i)
    if (sim->gps[i].valid) extend(&b, sim->gps[i].pos);
  if (est != NULL)
    for (size_t i = 0; i < est->len; ++i) extend(&b, est->data[i].pos);
  if (b.min_x > b.max_x) { /* no data at all */
    b.min_x = b.min_y = 0.0;
    b.max_x = b.max_y = 1.0;
  }

  const double range_x = fmax(b.max_x - b.min_x, 1.0);
  const double range_y = fmax(b.max_y - b.min_y, 1.0);
  const int aw = opt->width - 2 * opt->margin;
  const int ah = opt->height - 2 * opt->margin;
  const double avail_w = (double)(aw > 1 ? aw : 1);
  const double avail_h = (double)(ah > 1 ? ah : 1);

  viewport v;
  v.min_x = b.min_x;
  v.min_y = b.min_y;
  v.scale = fmin(avail_w / range_x, avail_h / range_y);
  v.off_x = (double)opt->margin + (avail_w - range_x * v.scale) / 2.0;
  v.off_y = (double)opt->margin + (avail_h - range_y * v.scale) / 2.0;
  v.height = opt->height;
  return v;
}

static kf_status polyline_open(FILE *fp, const char *color, double width) {
  OUT("<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"%.1f\" stroke-linejoin=\"round\" points=\"",
      color, width);
  return KF_OK;
}

static kf_status polyline_point(FILE *fp, const viewport *v, kf_point2d p) {
  OUT("%.1f,%.1f ", vp_x(v, p.x), vp_y(v, p.y));
  return KF_OK;
}

static kf_status polyline_close(FILE *fp) {
  OUT("\"/>\n");
  return KF_OK;
}

/* Truth path: needs at least two points to be a line. */
static kf_status draw_truth(FILE *fp, const kf_sim_result *sim, const viewport *v) {
  if (sim->truth_len < 2) return KF_OK;
  KF_TRY(polyline_open(fp, "#444444", 2.0));
  for (size_t i = 0; i < sim->truth_len; ++i) KF_TRY(polyline_point(fp, v, sim->truth[i].pos));
  return polyline_close(fp);
}

/* Draw estimates [begin, end) as one polyline. */
static kf_status draw_estimate_range(FILE *fp, const kf_estimates *est, size_t begin, size_t end,
                                     const viewport *v, const char *color, double width) {
  if (end - begin < 2) return KF_OK;
  KF_TRY(polyline_open(fp, color, width));
  for (size_t i = begin; i < end; ++i) KF_TRY(polyline_point(fp, v, est->data[i].pos));
  return polyline_close(fp);
}

static kf_status legend_entry(FILE *fp, int x, int y, const char *color, const char *label, bool dot) {
  if (dot) {
    OUT("<circle cx=\"%d\" cy=\"%d\" r=\"3\" fill=\"%s\"/>", x + 10, y - 4, color);
  } else {
    OUT("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"%s\" stroke-width=\"3\"/>", x, y - 4,
        x + 20, y - 4, color);
  }
  OUT("<text x=\"%d\" y=\"%d\" font-size=\"13\" fill=\"#222\">%s</text>\n", x + 28, y, label);
  return KF_OK;
}

kf_status kf_write_svg(FILE *fp, const kf_sim_result *sim, const kf_estimates *estimates,
                       const kf_svg_options *options) {
  if (fp == NULL || sim == NULL) return KF_ERR_INVALID_ARG;
  if ((sim->truth_len > 0 && sim->truth == NULL) || (sim->gps_len > 0 && sim->gps == NULL))
    return KF_ERR_INVALID_ARG;
  const kf_estimates empty = {NULL, 0};
  const kf_estimates *est = (estimates != NULL) ? estimates : &empty;
  if (est->len > 0 && est->data == NULL) return KF_ERR_INVALID_ARG;

  const kf_svg_options opt = (options != NULL) ? *options : kf_svg_options_default();
  if (opt.width <= 0 || opt.height <= 0 || opt.margin < 0) return KF_ERR_INVALID_ARG;

  const viewport v = make_viewport(sim, est, &opt);

  OUT("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n",
      opt.width, opt.height, opt.width, opt.height);
  OUT("<rect width=\"100%%\" height=\"100%%\" fill=\"#ffffff\"/>\n");
  OUT("<text x=\"%d\" y=\"28\" font-family=\"sans-serif\" font-size=\"16\" font-weight=\"bold\" "
      "fill=\"#222\">GPS + Kalman filter (simulated car, local metres)</text>\n",
      opt.margin);
  OUT("<g font-family=\"sans-serif\">\n");

  /* True path. */
  KF_TRY(draw_truth(fp, sim, &v));

  /* Raw GPS fixes. */
  if (opt.show_gps) {
    for (size_t i = 0; i < sim->gps_len; ++i) {
      const kf_gps_measurement *g = &sim->gps[i];
      if (!g->valid) continue;
      OUT("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"2.5\" fill=\"#e03131\" fill-opacity=\"0.6\"/>\n",
          vp_x(&v, g->pos.x), vp_y(&v, g->pos.y));
    }
  }

  /* Kalman estimate (blue), with the GPS-outage stretches overdrawn in orange. */
  KF_TRY(draw_estimate_range(fp, est, 0, est->len, &v, "#1971c2", 2.0));

  size_t run_start = 0;
  bool in_run = false;
  for (size_t i = 0; i < est->len; ++i) {
    const bool lost = kf_is_in_windows(est->data[i].t, sim->dropouts, sim->dropouts_len);
    if (lost && !in_run) {
      run_start = i;
      in_run = true;
    } else if (!lost && in_run) {
      KF_TRY(draw_estimate_range(fp, est, run_start, i, &v, "#f08c00", 3.0));
      in_run = false;
    }
  }
  if (in_run) KF_TRY(draw_estimate_range(fp, est, run_start, est->len, &v, "#f08c00", 3.0));

  /* Legend. */
  const int ly = opt.height - 20;
  KF_TRY(legend_entry(fp, opt.margin, ly - 54, "#444444", "True path", false));
  KF_TRY(legend_entry(fp, opt.margin, ly - 36, "#e03131", "Noisy GPS fixes", true));
  KF_TRY(legend_entry(fp, opt.margin, ly - 18, "#1971c2", "Kalman estimate", false));
  KF_TRY(legend_entry(fp, opt.margin, ly, "#f08c00", "Estimate during GPS outage", false));

  OUT("</g>\n</svg>\n");
  return KF_OK;
}

kf_status kf_write_svg_file(const char *path, const kf_sim_result *sim,
                            const kf_estimates *estimates, const kf_svg_options *options) {
  if (path == NULL) return KF_ERR_INVALID_ARG;
  FILE *fp = fopen(path, "w");
  if (fp == NULL) return KF_ERR_IO;
  kf_status st = kf_write_svg(fp, sim, estimates, options);
  if (fclose(fp) != 0 && st == KF_OK) st = KF_ERR_IO;
  return st;
}
