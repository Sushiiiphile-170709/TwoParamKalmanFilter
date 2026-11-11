#ifndef GPS_KALMAN_TYPES_H
#define GPS_KALMAN_TYPES_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
typedef struct { double x,y; } Point2D;
typedef struct { double start_s,end_s; } TimeWindow;
typedef struct { size_t step; double t; Point2D pos,vel; } TruthSample;
typedef struct { size_t step; double t; bool valid; Point2D pos; } GpsMeasurement;
typedef struct { size_t step; double t; Point2D pos,vel; double pos_std; bool used_gps; } EstimateSample;
typedef struct { double dt; TruthSample *truth; size_t truth_count, truth_capacity; GpsMeasurement *gps; size_t gps_count,gps_capacity; TimeWindow *dropouts; size_t dropout_count; } SimulationResult;
void simulation_result_free(SimulationResult *r);
#endif
