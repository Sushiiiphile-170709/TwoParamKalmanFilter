#ifndef GPS_KALMAN_CSV_WRITER_H
#define GPS_KALMAN_CSV_WRITER_H

/**
 * @file csv_writer.h
 * @brief Export truth, GPS and filter output as CSV for plotting elsewhere
 *        (Excel, Python, gnuplot...).
 *
 * One row per simulation step. Columns:
 *   t,true_x,true_y,true_vx,true_vy,gps_valid,gps_x,gps_y,
 *   est_x,est_y,est_vx,est_vy,est_pos_std,used_gps
 * Cells that do not exist for a step (no GPS epoch, filter not started yet)
 * are left empty. Numbers use three decimals.
 */

#include <stdio.h>

#include "status.h"
#include "types.h"

/** Write the CSV to an open stream. `estimates` may be NULL (= none). */
kf_status kf_write_results_csv(FILE *fp, const kf_sim_result *sim,
                               const kf_estimates *estimates);

/** Write the CSV to a file. @return KF_ERR_IO if it cannot be written. */
kf_status kf_write_results_csv_file(const char *path, const kf_sim_result *sim,
                                    const kf_estimates *estimates);

#endif /* GPS_KALMAN_CSV_WRITER_H */
