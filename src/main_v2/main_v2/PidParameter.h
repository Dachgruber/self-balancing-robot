/* Self balancing Robot via Stepper Motor with microstepping and Digital  Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
    modified by : Cornelius Brütt in 2023
*/
#ifndef  PidParameter_h
#define  PidParameter_h
#include "PidParameter.h"

//  PidParameter Motor
struct PidParameter {
  float          K = 5.0;
  float          Kp =  9.4282 ;
  float          Ki =  5.5223;
  float          Kd  = 0.145;
  float          Ka = 0.1220;
  float          Kx = 0.0; //
};
//  PidParameter Position
struct PidParameterPos {
  float          K = 1.0;
  float          Kp = 0.0;
  float          Ki = 0.0;
  float          Kd  = 0.08 ;
  float          Ka = 0.0 ;
  float          Kx = 0.0;
};
#endif

//Step=  1 Ind= 0 av= 1.5500 besterr 0.0300 P0 9.4282 P1 5.5223 P2 0.1445 P3 0.1220 P4 0.03  P5 0.00 P6 0.02 P7 0.0000
//Step= 1 Ind= 0 av= 5.1000 besterr 0.0267 P0 0.0315  P1 0.0015 P2 0.0000 P3 0.0000 P4 9.43 P5 5.52 P6 0.14 P7 0.1220 
//Step= 1 Ind=  1 av= 4.6900 besterr 0.0133 P0 0.0336 P1 0.0025 P2 0.0010 P3 0.0000 P4 9.43 P5 5.52  P6 0.14 P7 0.1220 
