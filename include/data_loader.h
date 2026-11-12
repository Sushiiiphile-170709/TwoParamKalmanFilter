#ifndef GPS_KALMAN_DATA_LOADER_H
#define GPS_KALMAN_DATA_LOADER_H

/**
 * @file data_loader.h
 * @brief Reads the mock-data files in data/: the route and the GPS outages.
 *
 * Both formats are plain CSV. Blank lines and lines starting with '#' are
 * ignored, and one optional header line (it starts with a letter) is skipped.
 * Lines are limited to 254 characters.
 *
 *   route:     duration_s,accel_mps2,yaw_rate_rad_s
 *   dropouts:  start_s,end_s
 *
 * On a PARSE error, *err_line (if not NULL) receives the 1-based line number
 * of the first bad line. Returned arrays are heap allocated; release them
 * with free().
 */

#include <stddef.h>

#include "simulator.h"
#include "status.h"
#include "types.h"

/**
 * Load route segments. At least one segment is required. Durations must be
 * finite and >= 0, and every value must be finite.
 */
kf_status kf_load_route_csv(const char *path, kf_segment **out, size_t *out_len,
                            size_t *err_line);

/**
 * Load GPS outage windows. An empty file is valid and means "no outages"
 * (*out is NULL, *out_len is 0). Each window needs start_s <= end_s.
 */
kf_status kf_load_dropouts_csv(const char *path, kf_time_window **out,
                               size_t *out_len, size_t *err_line);

#endif /* GPS_KALMAN_DATA_LOADER_H */
