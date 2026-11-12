#ifndef GPS_KALMAN_RNG_H
#define GPS_KALMAN_RNG_H

/**
 * @file rng.h
 * @brief Seedable random numbers for the GPS noise.
 *
 * The C standard library has no portable, reproducible Gaussian generator, so
 * this module carries its own:
 *
 *  - the 64-bit Mersenne Twister (MT19937-64), same algorithm and constants
 *    as C++ std::mt19937_64, so a given seed produces the same stream;
 *  - uniform doubles in [0, 1) built the way libstdc++ does it;
 *  - normal deviates from Marsaglia's polar method, again like libstdc++.
 *
 * That is what lets this C port reproduce the numbers of the original C++
 * program for the same seed. The same seed always gives the same data.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define KF_RNG_STATE_WORDS 312

typedef struct kf_rng {
  uint64_t mt[KF_RNG_STATE_WORDS];
  size_t index;
  bool has_saved; /**< the polar method yields numbers in pairs */
  double saved;
} kf_rng;

/** Seed the generator. Any 64-bit seed is valid. */
void kf_rng_seed(kf_rng *rng, uint64_t seed);

/** Next raw 64-bit output of MT19937-64. */
uint64_t kf_rng_next_u64(kf_rng *rng);

/** Uniform double in [0, 1). */
double kf_rng_uniform01(kf_rng *rng);

/** Normal deviate with the given mean and standard deviation. */
double kf_rng_normal(kf_rng *rng, double mean, double stddev);

#endif /* GPS_KALMAN_RNG_H */
