#include "test_suites.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}


int main(void) {
  UNITY_BEGIN();
  run_core_tests();
  run_rng_tests();
  run_matrix_tests();
  run_geo_tests();
  run_kalman_filter_tests();
  run_simulator_tests();
  run_metrics_tests();
  run_tracker_tests();
  run_io_tests();
  run_data_loader_tests();
  return UNITY_END();
}
