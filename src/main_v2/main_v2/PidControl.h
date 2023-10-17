/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
    modified by : Cornelius Brütt in 2023
*/
#ifndef   PidControl_h
#define  PidControl_h
#include "Arduino.h"

#include   "PidParameter.h"

/**********************************************************************/
class   PidControl
/**********************************************************************/
{
   public:

    PidControl(float K, float Kp, float Ki, float Kd, float iKa   , float iKx) ;
    PidControl(PidParameter      Params) ;
    PidControl(PidParameterPos     Params) ;
    PidControl* getInstance();

    /* C++ Overloading //   changePIDparams = different PidParameter !!!!
      An overloaded declaration   is a declaration that is declared with the same
      name as a previously declared   declaration in the same scope, except that
      both declarations have different   arguments and obviously different
      definition (implementation).
    */
     void     changePIDparams(PidParameter Params);
    void     changePIDparams(PidParameterPos   Params);

    float    calculate (float iAngle, float isetPoint );
    float     getSteps ();
    void     reset ();
    void     test      ( );
    float     DeltaKp(float iError);
    float    eSpeed;
    float    pTerm;
    float     iTerm;
    float    dTerm;
    float    error;
    float    integrated_error;
    // volatile bool  DirForward;
    float    Last_error;

  protected:
     struct   PidParameter params;
    float    LastK;
    float    K;
     float    Ki;
    float    Kd;
    float    Kp;
    float    Ka;
     float    Kx;
    //   float    Last_angle;
    float    timeChange ;
     unsigned long Last_time;
    unsigned long Now;
    int      ptr;
     PidControl* pPID;
    bool first ;
};

#endif
