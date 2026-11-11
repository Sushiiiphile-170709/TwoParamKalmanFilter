#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "kalman_filter.h"
#include "geo.h"
#include "simulator.h"
#include "tracker.h"
#define T(x) do{if(!(x)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);return 1;}}while(0)
static int test_kf(void){KalmanConfig c=kalman_default_config();KalmanFilter f;T(kalman_init(&f,&c)==KF_OK);T(kalman_predict(&f,1)==KF_ERR_STATE);T(kalman_initialize(&f,0,0,1,2)==KF_OK);T(kalman_predict(&f,2)==KF_OK);T(fabs(f.x[0]-2)<1e-9&&fabs(f.x[1]-4)<1e-9);T(kalman_update(&f,2,4)==KF_OK);return 0;}static int test_geo(void){LocalFrame f;LatLon o={45,10};T(local_frame_init(&f,o)==0);Point2D p=geo_to_local(&f,o);T(fabs(p.x)<1e-9&&fabs(p.y)<1e-9);LatLon q=local_to_geo(&f,(Point2D){100,200});Point2D z=geo_to_local(&f,q);T(fabs(z.x-100)<1e-6&&fabs(z.y-200)<1e-6);return 0;}static int test_sim(void){SimulatorConfig c=simulator_default_config();size_t rn,dn;const Segment*r=simulator_default_route(&rn);c.dropouts=simulator_default_dropouts(&dn);c.dropout_count=dn;SimulationResult s;T(simulator_run(&c,r,rn,&s)==0);T(s.truth_count>1000&&s.gps_count>100);EstimateSample*e;size_t n;KalmanConfig k=kalman_default_config();T(tracker_run(&s,&k,&e,&n)==0);T(n>0&&n<=s.truth_count);free(e);simulation_result_free(&s);return 0;}int main(void){int rc=test_kf()||test_geo()||test_sim();if(!rc)puts("All tests passed");return rc;}
