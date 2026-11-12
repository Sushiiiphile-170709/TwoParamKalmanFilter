#include "rng.h"

#include <math.h>

/* MT19937-64 parameters (Matsumoto & Nishimura), as in std::mt19937_64. */
#define MT_N KF_RNG_STATE_WORDS
#define MT_M 156
#define MT_MATRIX_A 0xB5026F5AA96619E9ULL
#define MT_UPPER_MASK 0xFFFFFFFF80000000ULL
#define MT_LOWER_MASK 0x000000007FFFFFFFULL
#define MT_INIT_MULT 6364136223846793005ULL

void kf_rng_seed(kf_rng *rng, uint64_t seed) {
  rng->mt[0] = seed;
  for (size_t i = 1; i < MT_N; ++i) {
    const uint64_t prev = rng->mt[i - 1];
    rng->mt[i] = MT_INIT_MULT * (prev ^ (prev >> 62)) + (uint64_t)i;
  }
  rng->index = MT_N; /* forces a twist on the first call */
  rng->has_saved = false;
  rng->saved = 0.0;
}

static void twist(kf_rng *rng) {
  for (size_t i = 0; i < MT_N; ++i) {
    const uint64_t x = (rng->mt[i] & MT_UPPER_MASK) | (rng->mt[(i + 1) % MT_N] & MT_LOWER_MASK);
    uint64_t xa = x >> 1;
    if (x & 1ULL) xa ^= MT_MATRIX_A;
    rng->mt[i] = rng->mt[(i + MT_M) % MT_N] ^ xa;
  }
  rng->index = 0;
}

uint64_t kf_rng_next_u64(kf_rng *rng) {
  if (rng->index >= MT_N) twist(rng);
  uint64_t x = rng->mt[rng->index++];
  x ^= (x >> 29) & 0x5555555555555555ULL;
  x ^= (x << 17) & 0x71D67FFFEDA60000ULL;
  x ^= (x << 37) & 0xFFF7EEE000000000ULL;
  x ^= (x >> 43);
  return x;
}

double kf_rng_uniform01(kf_rng *rng) {
  /* One 64-bit draw scaled by 2^-64, like std::generate_canonical<double, 53>
   * on a 64-bit engine. Rounding can land exactly on 1.0, so step back. */
  double u = (double)kf_rng_next_u64(rng) / 18446744073709551616.0;
  if (u >= 1.0) u = nextafter(1.0, 0.0);
  return u;
}

double kf_rng_normal(kf_rng *rng, double mean, double stddev) {
  double value;
  if (rng->has_saved) {
    rng->has_saved = false;
    value = rng->saved;
  } else {
    double x, y, r2;
    do {
      x = 2.0 * kf_rng_uniform01(rng) - 1.0;
      y = 2.0 * kf_rng_uniform01(rng) - 1.0;
      r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    const double mult = sqrt(-2.0 * log(r2) / r2);
    rng->saved = x * mult;
    rng->has_saved = true;
    value = y * mult;
  }
  return value * stddev + mean;
}
