#include "status.h"

const char *kf_status_str(kf_status status) {
  switch (status) {
    case KF_OK:                  return "ok";
    case KF_ERR_INVALID_ARG:     return "invalid argument";
    case KF_ERR_NOT_INITIALIZED: return "not initialized";
    case KF_ERR_SINGULAR:        return "singular matrix";
    case KF_ERR_DIM_MISMATCH:    return "matrix dimension mismatch";
    case KF_ERR_OUT_OF_RANGE:    return "index out of range";
    case KF_ERR_NO_MEMORY:       return "out of memory";
    case KF_ERR_IO:              return "I/O error";
    case KF_ERR_PARSE:           return "parse error";
  }
  return "unknown status";
}
