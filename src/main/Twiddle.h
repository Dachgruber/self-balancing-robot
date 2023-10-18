/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
PID   auto Tuning with Twiddle

    Twiddle is an algorithm that tries to find a   good choice of parameters for an algorithm. 
    Also known as Hill climbing,   it is an analogy to a mountaineer who looks for the summit in dense 
    fog   and steers his steps as steeply uphill as possible. If it only goes down in all   directions, 
    he has arrived at a summit.The Twiddle algorithm is used for   auto tuning of PID parameter. 
    First of all, parameters can be tested with   a manual tuning with a potentiometer.
*/
#ifndef Twiddle_h
#define  Twiddle_h
#include   "Arduino.h"
//**********************************************************************   /
class Twiddle
///**********************************************************************/
{
   public:
    Twiddle( int anzParams, float p0 , float p1, float p2, float p3,   float p4 , float p5, float p6, float p7,
             float dp0, float dp1, float   dp2, float dp3 , float dp4, float dp5, float dp6 , float dp7)  ; // Constructor
     Twiddle(int anzParams, float iparams[], float idparams[]  )  ; // Constructor

     float next(float error, float &p0, float &p1, float &p2, float &p3, float &p4,   float &p5, float &p6, float &p7);
    void calcCost(float average);
    void   logging();
    float params[8];
    float dparams[8];
    float besterr;
     float lastcost;
    float average;
    int   index ;
    int   AnzParams   = 8;
    float sum_average;
    unsigned int cnt_average;
    int   nextStep;
     int AnzahlElements = 8;


};
#endif
