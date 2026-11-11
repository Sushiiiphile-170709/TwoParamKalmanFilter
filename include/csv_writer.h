#ifndef GPS_KALMAN_CSV_WRITER_H
#define GPS_KALMAN_CSV_WRITER_H
#include "types.h"
int write_results_csv_file(const char*,const SimulationResult*,const EstimateSample*,size_t);
#endif
