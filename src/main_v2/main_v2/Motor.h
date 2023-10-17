/* Self balancing Robot via Stepper Motor with microstepping and Digital  Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
*/
#ifndef  Motor_h
#define  Motor_h
#include "Arduino.h"
#include "PidControl.h"
#include  "DuePWMmod.h"

// ------------------------------------------------------------------------
class  Motor
// ------------------------------------------------------------------------

{  public:
    Motor(DuePWMmod* ipwm,  int iPinDir, int iPinStep,
          int  iPinMs1, int iPinMs2, int iPinSleep, char iMotorSide);

    volatile bool  DirForward;

   // struct PidParameter params;
    void init() ;
    Motor* getInstance();
    void SleepMode ( );
    void RunMode ( );
    void toggleMode ( );
    float Run(float Steps);

    // Four different  step resolutions: full-step, half-step, 1/4-step, and 1/8-step
    void MsFull  ( );
    void MsHalf ( );
    void MsQuater ( );
    void MsMicrostep (  );

    int _Ms1, _Ms2, _PinDir, _PinStep, _PinSleep;
    int _Divisor;
    Motor* pMotor;

    unsigned long  lastTime;
    char          _MotorSide;
    DuePWMmod     *ptrpwm;


  private:
    bool MotorMode;
    int  ptr;
};
#endif
