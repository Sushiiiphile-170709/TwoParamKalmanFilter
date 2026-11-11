#ifndef GPS_KALMAN_SVG_PLOT_H
#define GPS_KALMAN_SVG_PLOT_H
#include "types.h"
typedef struct { int width,height,margin; bool show_gps; } SvgOptions;
SvgOptions svg_default_options(void); int write_svg_file(const char*,const SimulationResult*,const EstimateSample*,size_t,const SvgOptions*);
#endif
