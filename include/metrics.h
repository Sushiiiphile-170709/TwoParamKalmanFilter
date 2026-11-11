#ifndef GPS_KALMAN_METRICS_H
#define GPS_KALMAN_METRICS_H
#include "types.h"
typedef struct { size_t count; double rmse,mean,max; } ErrorStats;
double point_distance(Point2D,Point2D); ErrorStats compute_stats(const double*,size_t); ErrorStats gps_error_stats(const SimulationResult*); ErrorStats estimate_error_stats(const SimulationResult*,const EstimateSample*,size_t,int (*keep)(const TruthSample*,void*),void*);
#endif
