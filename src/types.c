#include "types.h"

#include <stdlib.h>

void kf_sim_result_free(kf_sim_result *result) {
  if (result == NULL) return;
  free(result->truth);
  free(result->gps);
  free(result->dropouts);
  result->dt = 0.0;
  result->truth = NULL;
  result->truth_len = 0;
  result->gps = NULL;
  result->gps_len = 0;
  result->dropouts = NULL;
  result->dropouts_len = 0;
}

void kf_estimates_free(kf_estimates *estimates) {
  if (estimates == NULL) return;
  free(estimates->data);
  estimates->data = NULL;
  estimates->len = 0;
}
