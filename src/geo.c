#include "geo.h"

#include <math.h>

double kf_deg_to_rad(double deg) { return deg * KF_PI / 180.0; }
double kf_rad_to_deg(double rad) { return rad * 180.0 / KF_PI; }

kf_status kf_local_frame_init(kf_local_frame *frame, kf_latlon origin) {
  if (frame == NULL) return KF_ERR_INVALID_ARG;
  /* Written as "not inside" so that NaN fails the test too. */
  if (!(origin.lat_deg > -90.0 && origin.lat_deg < 90.0)) return KF_ERR_INVALID_ARG;
  if (!(origin.lon_deg >= -180.0 && origin.lon_deg <= 180.0)) return KF_ERR_INVALID_ARG;
  frame->origin = origin;
  frame->cos_lat0 = cos(kf_deg_to_rad(origin.lat_deg));
  return KF_OK;
}

kf_point2d kf_local_frame_to_local(const kf_local_frame *frame, kf_latlon p) {
  kf_point2d out;
  out.x = KF_EARTH_RADIUS_M * kf_deg_to_rad(p.lon_deg - frame->origin.lon_deg) * frame->cos_lat0;
  out.y = KF_EARTH_RADIUS_M * kf_deg_to_rad(p.lat_deg - frame->origin.lat_deg);
  return out;
}

kf_latlon kf_local_frame_to_geodetic(const kf_local_frame *frame, kf_point2d p) {
  kf_latlon out;
  out.lat_deg = frame->origin.lat_deg + kf_rad_to_deg(p.y / KF_EARTH_RADIUS_M);
  out.lon_deg = frame->origin.lon_deg + kf_rad_to_deg(p.x / (KF_EARTH_RADIUS_M * frame->cos_lat0));
  return out;
}
