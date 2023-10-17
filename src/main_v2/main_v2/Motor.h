/* Self balancing Robot via Stepper Motor with microstepping and Digital  Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
*/
#ifndef  Motor_h
#define  Motor_h
#include "Arduino.h"
#include "PidControl.h"
#include  "PWM.h"

// ------------------------------------------------------------------------
class  Motor
// ------------------------------------------------------------------------

{  public:
    Motor(int iPinDir, int iPinStep,int iMotorSide); //removed the pwm, sleep and MS1-2 params

    volatile bool  DirForward;

   // struct PidParameter params;
    void init() ;
    Motor* getInstance();
    void SleepMode ( );
    void RunMode ( );
    void toggleMode ( );
    float Run(float Steps);



    int _PinDir, _PinStep;
    int _Divisor;
    Motor* pMotor;

    unsigned long  lastTime;
    char          _MotorSide;
    //DuePWMmod     *ptrpwm;


  private:
    bool MotorMode;
    int  ptr;
};
#endif
