#ifndef GPS_KALMAN_STATUS_H
#define GPS_KALMAN_STATUS_H

/**
 * @file status.h
 * @brief Error codes returned by every fallible function in the project.
 *
 * C has no exceptions, so functions that can fail return a kf_status and hand
 * their result back through an out-parameter. On any error other than
 * KF_OK the out-parameter is left unchanged (or zeroed, where a function
 * documents that), so a failed call never leaves half-built data behind.
 */

typedef enum kf_status {
  KF_OK = 0,
  KF_ERR_INVALID_ARG,     /**< NULL pointer, NaN, negative size, bad range... */
  KF_ERR_NOT_INITIALIZED, /**< the object needs an init/start call first      */
  KF_ERR_SINGULAR,        /**< matrix cannot be inverted                      */
  KF_ERR_DIM_MISMATCH,    /**< matrix shapes do not fit the operation         */
  KF_ERR_OUT_OF_RANGE,    /**< index outside of its array                     */
  KF_ERR_NO_MEMORY,       /**< allocation failed or a size overflowed         */
  KF_ERR_IO,              /**< a file could not be opened, read or written    */
  KF_ERR_PARSE            /**< a data file has bad content                    */
} kf_status;

/** Short, constant description of a status code. Never returns NULL. */
const char *kf_status_str(kf_status status);

/** Evaluate `expr`; if it is not KF_OK, return that status from the caller. */
#define KF_TRY(expr)                                    \
  do {                                                  \
    const kf_status kf_try_status_ = (expr);            \
    if (kf_try_status_ != KF_OK) return kf_try_status_; \
  } while (0)

#endif /* GPS_KALMAN_STATUS_H */
