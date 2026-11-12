#include <math.h>

#include "geo.h"
#include "test_common.h"
#include "test_suites.h"

static kf_local_frame frame_at(double lat, double lon) {
  kf_local_frame f;
  kf_latlon origin = {lat, lon};
  ASSERT_STATUS(KF_OK, kf_local_frame_init(&f, origin));
  return f;
}

static void test_origin_maps_to_zero(void) {
  const kf_local_frame f = frame_at(45.0, 10.0);
  const kf_point2d p = kf_local_frame_to_local(&f, (kf_latlon){45.0, 10.0});
  ASSERT_NEAR(0.0, p.x, 1e-9);
  ASSERT_NEAR(0.0, p.y, 1e-9);
}

static void test_one_degree_north_is_about_111_km(void) {
  const kf_local_frame f = frame_at(45.0, 10.0);
  const kf_point2d p = kf_local_frame_to_local(&f, (kf_latlon){46.0, 10.0});
  ASSERT_NEAR(KF_EARTH_RADIUS_M * KF_PI / 180.0, p.y, 1e-6);
  ASSERT_NEAR(0.0, p.x, 1e-6);
}

static void test_east_distance_shrinks_with_latitude(void) {
  const kf_local_frame equator = frame_at(0.0, 0.0);
  const kf_local_frame sixty = frame_at(60.0, 0.0);
  const double at_equator = kf_local_frame_to_local(&equator, (kf_latlon){0.0, 1.0}).x;
  const double at_sixty = kf_local_frame_to_local(&sixty, (kf_latlon){60.0, 1.0}).x;
  ASSERT_NEAR(0.5, at_sixty / at_equator, 1e-9); /* cos(60 deg) = 0.5 */
}

static void test_round_trip_local_geodetic_local(void) {
  const kf_local_frame f = frame_at(45.0, 10.0);
  const kf_point2d original = {1234.5, -678.9};
  const kf_point2d back = kf_local_frame_to_local(&f, kf_local_frame_to_geodetic(&f, original));
  ASSERT_NEAR(original.x, back.x, 1e-6);
  ASSERT_NEAR(original.y, back.y, 1e-6);
}

static void test_degree_radian_conversion(void) {
  ASSERT_NEAR(KF_PI, kf_deg_to_rad(180.0), 1e-15);
  ASSERT_NEAR(90.0, kf_rad_to_deg(KF_PI / 2.0), 1e-12);
}

static void test_invalid_origin_is_rejected(void) {
  kf_local_frame f;
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_local_frame_init(&f, (kf_latlon){90.0, 0.0}));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_local_frame_init(&f, (kf_latlon){-95.0, 0.0}));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_local_frame_init(&f, (kf_latlon){0.0, 200.0}));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_local_frame_init(&f, (kf_latlon){NAN, 0.0}));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_local_frame_init(&f, (kf_latlon){0.0, NAN}));
  ASSERT_STATUS(KF_ERR_INVALID_ARG, kf_local_frame_init(NULL, (kf_latlon){0.0, 0.0}));
  ASSERT_STATUS(KF_OK, kf_local_frame_init(&f, (kf_latlon){89.9, 180.0}));
}

void run_geo_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_origin_maps_to_zero);
  RUN_TEST(test_one_degree_north_is_about_111_km);
  RUN_TEST(test_east_distance_shrinks_with_latitude);
  RUN_TEST(test_round_trip_local_geodetic_local);
  RUN_TEST(test_degree_radian_conversion);
  RUN_TEST(test_invalid_origin_is_rejected);
}
