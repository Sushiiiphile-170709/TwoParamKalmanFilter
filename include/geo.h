#ifndef GPS_KALMAN_GEO_H
#define GPS_KALMAN_GEO_H
#include "types.h"
#define EARTH_RADIUS_M 6371000.0
typedef struct { double lat_deg,lon_deg; } LatLon; typedef struct { LatLon origin; double cos_lat0; } LocalFrame;
double deg_to_rad(double); double rad_to_deg(double); int local_frame_init(LocalFrame*,LatLon); Point2D geo_to_local(const LocalFrame*,LatLon); LatLon local_to_geo(const LocalFrame*,Point2D);
#endif
