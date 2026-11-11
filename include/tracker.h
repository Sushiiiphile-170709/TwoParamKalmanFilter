#ifndef GPS_KALMAN_TRACKER_H
#define GPS_KALMAN_TRACKER_H
#include "types.h"
#include "kalman_filter.h"
int tracker_run(const SimulationResult*,const KalmanConfig*,EstimateSample **out,size_t *out_count);
#endif
