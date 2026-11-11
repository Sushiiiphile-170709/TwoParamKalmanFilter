#ifndef GPS_KALMAN_SIMULATOR_H
#define GPS_KALMAN_SIMULATOR_H
#include "types.h"
typedef struct { double duration_s,accel_mps2,yaw_rate_rad_s; } Segment;
typedef struct { double dt_s,gps_period_s,gps_noise_std_m,initial_speed_mps,initial_heading_rad; Point2D initial_position; const TimeWindow *dropouts; size_t dropout_count; uint64_t seed; } SimulatorConfig;
SimulatorConfig simulator_default_config(void); const Segment *simulator_default_route(size_t *count); const TimeWindow *simulator_default_dropouts(size_t *count); bool is_in_windows(double t,const TimeWindow*,size_t); int simulator_run(const SimulatorConfig*,const Segment*,size_t,SimulationResult*);
#endif
