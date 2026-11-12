#ifndef GPS_KALMAN_SVG_PLOT_H
#define GPS_KALMAN_SVG_PLOT_H

/**
 * @file svg_plot.h
 * @brief Draws the trip as a self-contained SVG image. Open it in any web
 *        browser; no GUI library is involved.
 *
 * Dark line = true path, red dots = noisy GPS fixes, blue line = Kalman
 * estimate, orange line = estimate while GPS was lost.
 */

#include <stdbool.h>
#include <stdio.h>

#include "status.h"
#include "types.h"

typedef struct kf_svg_options {
  int width;
  int height;
  int margin;
  bool show_gps;
} kf_svg_options;

/** 900 x 700, margin 50, GPS dots shown. */
kf_svg_options kf_svg_options_default(void);

/** Write the SVG document to an open stream. `estimates` may be NULL. */
kf_status kf_write_svg(FILE *fp, const kf_sim_result *sim,
                       const kf_estimates *estimates,
                       const kf_svg_options *options);

/** Write the SVG to a file. `options` may be NULL for the defaults. */
kf_status kf_write_svg_file(const char *path, const kf_sim_result *sim,
                            const kf_estimates *estimates,
                            const kf_svg_options *options);

#endif /* GPS_KALMAN_SVG_PLOT_H */
