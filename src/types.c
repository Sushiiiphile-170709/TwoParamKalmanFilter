#include "types.h"
#include <stdlib.h>
void simulation_result_free(SimulationResult *r){if(!r)return;free(r->truth);free(r->gps);free(r->dropouts);*r=(SimulationResult){0};}
