/*
 * Random numbers. The "golden" values below were produced by C++
 * std::mt19937_64 and std::normal_distribution (libstdc++), so these tests
 * also pin down that the C generator matches the original program's data.
 */

#include <math.h>

#include "rng.h"
#include "test_common.h"
#include "test_suites.h"

static void test_mt19937_64_matches_the_standard_10000th_value(void) {
  /* The C++ standard requires the 10000th output for seed 5489 to be this. */
  kf_rng rng;
  kf_rng_seed(&rng, 5489);
  uint64_t v = 0;
  for (int i = 0; i < 10000; ++i) v = kf_rng_next_u64(&rng);
  TEST_ASSERT_EQUAL_UINT64(9981545732273789042ULL, v);
}

static void test_mt19937_64_first_outputs(void) {
  kf_rng rng;
  kf_rng_seed(&rng, 5489);
  TEST_ASSERT_EQUAL_UINT64(14514284786278117030ULL, kf_rng_next_u64(&rng));
  kf_rng_seed(&rng, 42);
  TEST_ASSERT_EQUAL_UINT64(13930160852258120406ULL, kf_rng_next_u64(&rng));
}

static void test_normal_matches_libstdcxx_for_seed_42(void) {
  const double expected[] = {3.5249413321042993, 6.4691021163646836, -2.8704740336013086,
                             1.9898869809189448, -9.5334267241523225, 5.5927752622873959};
  kf_rng rng;
  kf_rng_seed(&rng, 42);
  for (size_t i = 0; i < sizeof expected / sizeof expected[0]; ++i) {
    ASSERT_NEAR(expected[i], kf_rng_normal(&rng, 0.0, 5.0), 1e-12);
  }
}

static void test_normal_applies_mean_and_stddev(void) {
  const double expected[] = {11.745390333870949, 8.0548742446962507, 11.094619985297102,
                             12.910356321199769};
  kf_rng rng;
  kf_rng_seed(&rng, 7);
  for (size_t i = 0; i < sizeof expected / sizeof expected[0]; ++i) {
    ASSERT_NEAR(expected[i], kf_rng_normal(&rng, 10.0, 2.0), 1e-12);
  }
}

static void test_same_seed_gives_same_stream(void) {
  kf_rng a, b;
  kf_rng_seed(&a, 123456789);
  kf_rng_seed(&b, 123456789);
  for (int i = 0; i < 1000; ++i) TEST_ASSERT_EQUAL_UINT64(kf_rng_next_u64(&a), kf_rng_next_u64(&b));
}

static void test_reseeding_restarts_the_stream(void) {
  /* One normal draw leaves a second deviate saved; reseeding must drop it. */
  kf_rng used, fresh;
  kf_rng_seed(&used, 9);
  (void)kf_rng_normal(&used, 0.0, 1.0);
  (void)kf_rng_next_u64(&used);
  kf_rng_seed(&used, 9);
  kf_rng_seed(&fresh, 9);
  for (int i = 0; i < 5; ++i) {
    ASSERT_EXACT(kf_rng_normal(&fresh, 0.0, 1.0), kf_rng_normal(&used, 0.0, 1.0));
  }
  TEST_ASSERT_EQUAL_UINT64(kf_rng_next_u64(&fresh), kf_rng_next_u64(&used));
}

static void test_different_seeds_give_different_streams(void) {
  kf_rng a, b;
  kf_rng_seed(&a, 1);
  kf_rng_seed(&b, 2);
  TEST_ASSERT_TRUE(kf_rng_next_u64(&a) != kf_rng_next_u64(&b));
}

static void test_uniform_stays_in_unit_interval_with_mean_half(void) {
  kf_rng rng;
  kf_rng_seed(&rng, 2024);
  double sum = 0.0;
  const int n = 100000;
  for (int i = 0; i < n; ++i) {
    const double u = kf_rng_uniform01(&rng);
    TEST_ASSERT_TRUE(u >= 0.0 && u < 1.0);
    sum += u;
  }
  ASSERT_NEAR(0.5, sum / n, 0.01);
}

static void test_normal_has_requested_mean_and_spread(void) {
  kf_rng rng;
  kf_rng_seed(&rng, 31337);
  const int n = 200000;
  double sum = 0.0, sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double x = kf_rng_normal(&rng, 3.0, 2.0);
    sum += x;
    sum_sq += x * x;
  }
  const double mean = sum / n;
  const double var = sum_sq / n - mean * mean;
  ASSERT_NEAR(3.0, mean, 0.03);
  ASSERT_NEAR(2.0, sqrt(var), 0.03);
}

void run_rng_tests(void) {
  BEGIN_TEST_FILE();
  RUN_TEST(test_mt19937_64_matches_the_standard_10000th_value);
  RUN_TEST(test_mt19937_64_first_outputs);
  RUN_TEST(test_normal_matches_libstdcxx_for_seed_42);
  RUN_TEST(test_normal_applies_mean_and_stddev);
  RUN_TEST(test_same_seed_gives_same_stream);
  RUN_TEST(test_reseeding_restarts_the_stream);
  RUN_TEST(test_different_seeds_give_different_streams);
  RUN_TEST(test_uniform_stays_in_unit_interval_with_mean_half);
  RUN_TEST(test_normal_has_requested_mean_and_spread);
}
