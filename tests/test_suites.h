#ifndef KF_TEST_SUITES_H
#define KF_TEST_SUITES_H

/* One entry point per test file; test_main.c calls them in order. */
void run_core_tests(void);
void run_rng_tests(void);
void run_matrix_tests(void);
void run_geo_tests(void);
void run_kalman_filter_tests(void);
void run_simulator_tests(void);
void run_metrics_tests(void);
void run_tracker_tests(void);
void run_io_tests(void);
void run_data_loader_tests(void);

#endif /* KF_TEST_SUITES_H */
