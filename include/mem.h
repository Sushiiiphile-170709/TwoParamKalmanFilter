#ifndef GPS_KALMAN_MEM_H
#define GPS_KALMAN_MEM_H

/**
 * @file mem.h
 * @brief Overflow-checked array allocation.
 */

#include <stddef.h>

#include "status.h"

/**
 * Allocate a zeroed array of `count` elements of `elem_size` bytes.
 *
 * `count == 0` is allowed and yields *out == NULL with KF_OK.
 * @return KF_ERR_NO_MEMORY if count * elem_size overflows or the allocation
 *         fails (*out is set to NULL); KF_ERR_INVALID_ARG if out is NULL.
 */
kf_status kf_alloc_array(void **out, size_t count, size_t elem_size);

/**
 * Double the capacity of a heap array (starting at 16 elements).
 * On failure the old array is still valid and still owned by the caller.
 */
kf_status kf_grow_array(void **array, size_t *capacity, size_t elem_size);

#endif /* GPS_KALMAN_MEM_H */
