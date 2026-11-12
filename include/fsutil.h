#ifndef GPS_KALMAN_FSUTIL_H
#define GPS_KALMAN_FSUTIL_H

/**
 * @file fsutil.h
 * @brief Small POSIX filesystem helpers for the demo program.
 */

#include <stddef.h>

#include "status.h"

/** Create a directory and any missing parents (like `mkdir -p`). */
kf_status kf_make_dirs(const char *path);

/**
 * Write "dir/name" into buf. Fails with KF_ERR_INVALID_ARG if it does not fit
 * (no silent truncation).
 */
kf_status kf_path_join(char *buf, size_t buf_size, const char *dir,
                       const char *name);

#endif /* GPS_KALMAN_FSUTIL_H */
