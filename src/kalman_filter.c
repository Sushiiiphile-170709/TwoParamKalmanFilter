#include "kalman_filter.h"

#include <math.h>

kf_config kf_config_default(void) {
  kf_config c;
  c.process_noise_accel_std = 1.0;
  c.measurement_noise_std = 5.0;
  c.initial_position_variance = 25.0;
  c.initial_velocity_variance = 100.0;
  return c;
}

kf_status kf_filter_init(kf_filter *filter, const kf_config *config) {
  if (filter == NULL) return KF_ERR_INVALID_ARG;
  const kf_config cfg = (config != NULL) ? *config : kf_config_default();

  /* Each test is written so that NaN fails it. */
  if (!(cfg.process_noise_accel_std >= 0.0) || !isfinite(cfg.process_noise_accel_std))
    return KF_ERR_INVALID_ARG;
  if (!(cfg.measurement_noise_std > 0.0) || !isfinite(cfg.measurement_noise_std))
    return KF_ERR_INVALID_ARG;
  if (!(cfg.initial_position_variance > 0.0) || !isfinite(cfg.initial_position_variance))
    return KF_ERR_INVALID_ARG;
  if (!(cfg.initial_velocity_variance > 0.0) || !isfinite(cfg.initial_velocity_variance))
    return KF_ERR_INVALID_ARG;

  kf_filter f;
  f.config = cfg;
  f.initialized = false;
  KF_TRY(kf_mat_zero(&f.x, 4, 1));
  KF_TRY(kf_mat_zero(&f.p, 4, 4));
  *filter = f;
  return KF_OK;
}

kf_status kf_filter_start(kf_filter *filter, double px, double py, double vx, double vy) {
  if (filter == NULL) return KF_ERR_INVALID_ARG;
  if (!isfinite(px) || !isfinite(py) || !isfinite(vx) || !isfinite(vy)) return KF_ERR_INVALID_ARG;

  KF_MAT_AT(filter->x, 0, 0) = px;
  KF_MAT_AT(filter->x, 1, 0) = py;
  KF_MAT_AT(filter->x, 2, 0) = vx;
  KF_MAT_AT(filter->x, 3, 0) = vy;

  KF_TRY(kf_mat_zero(&filter->p, 4, 4));
  KF_MAT_AT(filter->p, 0, 0) = filter->config.initial_position_variance;
  KF_MAT_AT(filter->p, 1, 1) = filter->config.initial_position_variance;
  KF_MAT_AT(filter->p, 2, 2) = filter->config.initial_velocity_variance;
  KF_MAT_AT(filter->p, 3, 3) = filter->config.initial_velocity_variance;
  filter->initialized = true;
  return KF_OK;
}

bool kf_filter_is_initialized(const kf_filter *filter) {
  return filter != NULL && filter->initialized;
}

kf_status kf_filter_predict(kf_filter *filter, double dt) {
  if (filter == NULL) return KF_ERR_INVALID_ARG;
  if (!filter->initialized) return KF_ERR_NOT_INITIALIZED;
  if (!(dt >= 0.0) || !isfinite(dt)) return KF_ERR_INVALID_ARG;

  /* State transition F of the constant-velocity model. */
  kf_mat f;
  KF_TRY(kf_mat_identity(&f, 4));
  KF_MAT_AT(f, 0, 2) = dt;
  KF_MAT_AT(f, 1, 3) = dt;

  /* Process noise Q of a continuous white-noise-acceleration model, per axis:
   *     S * [ dt^3/3  dt^2/2 ]
   *         [ dt^2/2  dt     ]
   * with S = sigma_a^2 * (1 s). The noise injected over a given time does not
   * depend on how finely that time is cut into predict() calls. */
  const double s_int = filter->config.process_noise_accel_std * filter->config.process_noise_accel_std;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  kf_mat q;
  KF_TRY(kf_mat_zero(&q, 4, 4));
  KF_MAT_AT(q, 0, 0) = KF_MAT_AT(q, 1, 1) = s_int * dt3 / 3.0;
  KF_MAT_AT(q, 0, 2) = KF_MAT_AT(q, 2, 0) = s_int * dt2 / 2.0;
  KF_MAT_AT(q, 1, 3) = KF_MAT_AT(q, 3, 1) = s_int * dt2 / 2.0;
  KF_MAT_AT(q, 2, 2) = KF_MAT_AT(q, 3, 3) = s_int * dt;

  /* x' = F x     P' = F P F^T + Q. Results are committed only at the end. */
  kf_mat x_new, ft, fp, fpft, p_new;
  KF_TRY(kf_mat_mul(&f, &filter->x, &x_new));
  KF_TRY(kf_mat_transpose(&f, &ft));
  KF_TRY(kf_mat_mul(&f, &filter->p, &fp));
  KF_TRY(kf_mat_mul(&fp, &ft, &fpft));
  KF_TRY(kf_mat_add(&fpft, &q, &p_new));

  filter->x = x_new;
  filter->p = p_new;
  return KF_OK;
}

kf_status kf_filter_update(kf_filter *filter, double zx, double zy) {
  if (filter == NULL) return KF_ERR_INVALID_ARG;
  if (!filter->initialized) return KF_ERR_NOT_INITIALIZED;
  if (!isfinite(zx) || !isfinite(zy)) return KF_ERR_INVALID_ARG;

  /* Measurement model H: we observe position only. */
  kf_mat h;
  KF_TRY(kf_mat_zero(&h, 2, 4));
  KF_MAT_AT(h, 0, 0) = 1.0;
  KF_MAT_AT(h, 1, 1) = 1.0;

  const double r_var = filter->config.measurement_noise_std * filter->config.measurement_noise_std;
  kf_mat r;
  KF_TRY(kf_mat_zero(&r, 2, 2));
  KF_MAT_AT(r, 0, 0) = r_var;
  KF_MAT_AT(r, 1, 1) = r_var;

  kf_mat z, hx, innovation, ht, hp, hpht, s, s_inv;
  const double z_values[2] = {zx, zy};
  KF_TRY(kf_mat_from_values(&z, 2, 1, z_values, 2));
  KF_TRY(kf_mat_mul(&h, &filter->x, &hx));
  KF_TRY(kf_mat_sub(&z, &hx, &innovation));
  KF_TRY(kf_mat_transpose(&h, &ht));
  KF_TRY(kf_mat_mul(&h, &filter->p, &hp));
  KF_TRY(kf_mat_mul(&hp, &ht, &hpht));
  KF_TRY(kf_mat_add(&hpht, &r, &s));
  KF_TRY(kf_mat_invert(&s, &s_inv, KF_MAT_DEFAULT_PIVOT_TOL)); /* KF_ERR_SINGULAR: nothing changed */

  /* Kalman gain K = P H^T S^-1, then x' = x + K * innovation. */
  kf_mat pht, k, k_innov, x_new;
  KF_TRY(kf_mat_mul(&filter->p, &ht, &pht));
  KF_TRY(kf_mat_mul(&pht, &s_inv, &k));
  KF_TRY(kf_mat_mul(&k, &innovation, &k_innov));
  KF_TRY(kf_mat_add(&filter->x, &k_innov, &x_new));

  /* Joseph form P' = (I-KH) P (I-KH)^T + K R K^T keeps P symmetric and
   * positive semi-definite despite rounding. */
  kf_mat eye, kh, i_kh, i_kh_t, a1, a2, kr, k_t, kr_kt, p_new;
  KF_TRY(kf_mat_identity(&eye, 4));
  KF_TRY(kf_mat_mul(&k, &h, &kh));
  KF_TRY(kf_mat_sub(&eye, &kh, &i_kh));
  KF_TRY(kf_mat_transpose(&i_kh, &i_kh_t));
  KF_TRY(kf_mat_mul(&i_kh, &filter->p, &a1));
  KF_TRY(kf_mat_mul(&a1, &i_kh_t, &a2));
  KF_TRY(kf_mat_mul(&k, &r, &kr));
  KF_TRY(kf_mat_transpose(&k, &k_t));
  KF_TRY(kf_mat_mul(&kr, &k_t, &kr_kt));
  KF_TRY(kf_mat_add(&a2, &kr_kt, &p_new));

  filter->x = x_new;
  filter->p = p_new;
  return KF_OK;
}

double kf_filter_position_x(const kf_filter *f) { return KF_MAT_AT(f->x, 0, 0); }
double kf_filter_position_y(const kf_filter *f) { return KF_MAT_AT(f->x, 1, 0); }
double kf_filter_velocity_x(const kf_filter *f) { return KF_MAT_AT(f->x, 2, 0); }
double kf_filter_velocity_y(const kf_filter *f) { return KF_MAT_AT(f->x, 3, 0); }

double kf_filter_speed(const kf_filter *f) {
  return hypot(kf_filter_velocity_x(f), kf_filter_velocity_y(f));
}

double kf_filter_position_std(const kf_filter *f) {
  return sqrt(KF_MAT_AT(f->p, 0, 0) + KF_MAT_AT(f->p, 1, 1));
}

const kf_mat *kf_filter_covariance(const kf_filter *f) { return &f->p; }
