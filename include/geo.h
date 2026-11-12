#ifndef GPS_KALMAN_GEO_H
#define GPS_KALMAN_GEO_H

/**
 * @file geo.h
 * @brief Conversion between latitude/longitude and a local east/north frame
 *        in meters.
 *
 * Real receivers report latitude and longitude, but the filter works in
 * meters. This module uses an equirectangular (flat-earth) approximation
 * around a chosen origin: accurate to a few centimeters over a car trip of a
 * few kilometers, and wrong over long distances.
 */

#include "status.h"
#include "types.h"

#define KF_PI 3.14159265358979323846
#define KF_EARTH_RADIUS_M 6371000.0 /**< mean Earth radius (m) */

/** Geodetic position in degrees. */
typedef struct kf_latlon {
  double lat_deg;
  double lon_deg;
} kf_latlon;

/** Local tangent-plane frame anchored at a geodetic origin. */
typedef struct kf_local_frame {
  kf_latlon origin;
  double cos_lat0;
} kf_local_frame;

double kf_deg_to_rad(double deg);
double kf_rad_to_deg(double rad);

/**
 * Set up a frame around `origin`.
 * @return KF_ERR_INVALID_ARG if the latitude is not in (-90, 90) or the
 *         longitude is not in [-180, 180] (NaN is rejected too).
 */
kf_status kf_local_frame_init(kf_local_frame *frame, kf_latlon origin);

/** Geodetic -> local (x = east, y = north), in meters. */
kf_point2d kf_local_frame_to_local(const kf_local_frame *frame, kf_latlon p);

/** Local (meters) -> geodetic. */
kf_latlon kf_local_frame_to_geodetic(const kf_local_frame *frame, kf_point2d p);

#endif /* GPS_KALMAN_GEO_H */
