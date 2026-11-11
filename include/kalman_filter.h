#ifndef GPS_KALMAN_KALMAN_FILTER_H
#define GPS_KALMAN_KALMAN_FILTER_H
#include <stdbool.h>
typedef enum { KF_OK=0, KF_ERR_ARGUMENT, KF_ERR_STATE, KF_ERR_SINGULAR } KfStatus;
typedef struct { double process_noise_accel_std,measurement_noise_std,initial_position_variance,initial_velocity_variance; } KalmanConfig;
typedef struct { KalmanConfig config; double x[4]; double p[4][4]; bool initialized; } KalmanFilter;
KalmanConfig kalman_default_config(void); KfStatus kalman_init(KalmanFilter*,const KalmanConfig*); KfStatus kalman_initialize(KalmanFilter*,double,double,double,double); KfStatus kalman_predict(KalmanFilter*,double); KfStatus kalman_update(KalmanFilter*,double,double); double kalman_speed(const KalmanFilter*); double kalman_position_std(const KalmanFilter*);
#endif
