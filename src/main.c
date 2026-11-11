#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include "simulator.h"
#include "tracker.h"
#include "metrics.h"
#include "geo.h"
#include "csv_writer.h"
#include "svg_plot.h"
static void usage(const char*p){printf("Usage: %s [--seed N] [--gps-noise M] [--assumed-noise M] [--accel-std A] [--out-dir DIR]\n",p);}int main(int ac,char**av){SimulatorConfig sc=simulator_default_config();size_t rn,dn;const Segment*r=simulator_default_route(&rn);const TimeWindow*d=simulator_default_dropouts(&dn);sc.dropouts=d;sc.dropout_count=dn;KalmanConfig kc=kalman_default_config();const char*outdir="output";for(int i=1;i<ac;i++){if(!strcmp(av[i],"--help")||!strcmp(av[i],"-h")){usage(av[0]);return 0;}if(i+1>=ac){usage(av[0]);return 1;}char*end=NULL;const char*v=av[++i];if(!strcmp(av[i-1],"--seed"))sc.seed=strtoull(v,&end,10);else if(!strcmp(av[i-1],"--gps-noise"))sc.gps_noise_std_m=strtod(v,&end);else if(!strcmp(av[i-1],"--assumed-noise"))kc.measurement_noise_std=strtod(v,&end);else if(!strcmp(av[i-1],"--accel-std"))kc.process_noise_accel_std=strtod(v,&end);else if(!strcmp(av[i-1],"--out-dir")){outdir=v;continue;}else {usage(av[0]);return 1;}if(!end||*end){fprintf(stderr,"invalid value\n");return 1;}}if(kc.measurement_noise_std==5.0)kc.measurement_noise_std=sc.gps_noise_std_m>0?sc.gps_noise_std_m:1e-3;SimulationResult sim;if(simulator_run(&sc,r,rn,&sim)){fprintf(stderr,"simulation failed\n");return 1;}EstimateSample*est=NULL;size_t en=0;if(tracker_run(&sim,&kc,&est,&en)){fprintf(stderr,"tracking failed\n");simulation_result_free(&sim);return 1;}ErrorStats g=gps_error_stats(&sim),k=estimate_error_stats(&sim,est,en,NULL,NULL);printf("Simulated trip: %.1f s, %zu truth samples\nRaw GPS RMSE: %.2f m\nKalman RMSE: %.2f m\n",sim.truth[sim.truth_count-1].t,sim.truth_count,g.rmse,k.rmse);if(mkdir(outdir,0755)&&errno!=EEXIST)fprintf(stderr,"warning: cannot create %s\n",outdir);char csv[512],svg[512];snprintf(csv,sizeof csv,"%s/results.csv",outdir);snprintf(svg,sizeof svg,"%s/track.svg",outdir);if(write_results_csv_file(csv,&sim,est,en)||write_svg_file(svg,&sim,est,en,NULL))fprintf(stderr,"warning: output write failed\n");else printf("Wrote %s and %s\n",csv,svg);free(est);simulation_result_free(&sim);return 0;}
