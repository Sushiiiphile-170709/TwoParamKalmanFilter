#include "mem.h"

#include <stdint.h>
#include <stdlib.h>

kf_status kf_alloc_array(void **out, size_t count, size_t elem_size) {
  if (out == NULL) return KF_ERR_INVALID_ARG;
  *out = NULL;
  if (count == 0 || elem_size == 0) return KF_OK;
  if (count > SIZE_MAX / elem_size) return KF_ERR_NO_MEMORY;
  void *p = calloc(count, elem_size);
  if (p == NULL) return KF_ERR_NO_MEMORY;
  *out = p;
  return KF_OK;
}

kf_status kf_grow_array(void **array, size_t *capacity, size_t elem_size) {
  if (array == NULL || capacity == NULL || elem_size == 0) return KF_ERR_INVALID_ARG;
  const size_t new_cap = (*capacity == 0) ? 16 : *capacity * 2;
  if (new_cap < *capacity || new_cap > SIZE_MAX / elem_size) return KF_ERR_NO_MEMORY;
  void *p = realloc(*array, new_cap * elem_size);
  if (p == NULL) return KF_ERR_NO_MEMORY;
  *array = p;
  *capacity = new_cap;
  return KF_OK;
}
