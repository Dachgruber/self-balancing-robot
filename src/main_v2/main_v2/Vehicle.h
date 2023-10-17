/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
    modified by : Cornelius Brütt in 2023
*/

#ifndef   Vehicle_h
#define  Vehicle_h
#include "Motor.h"
#include "Arduino.h"
#include   "Config.h"
/**********************************************************************/
class   Vehicle
/**********************************************************************/
{
   public:
    Vehicle(Motor * MotorA, Motor * MotorB,
            PidControl   * PIDContropPIDController, PidControl * PIDContropPIDControllerPos);  // Constructor

    Motor *pMotorA;
    Motor *pMotorB;
    PidControl *pPIDController;
    PidControl *pPIDControllerPos;

    void Run(MpuYawPitchRoll YawPitchRoll,  int &PositionA, int &PositionB);
    void init();

     void Stop( );

    float DeltaPos       = 0.0;
    float DeltaForward   = 0.0;
    float DeltaTurning   = 0.0;
    float PositionAB     = 0.0;
    float HoldPosition   = 0.0;
    float StepsA         = 0.0;
    float StepsB         = 0.0;
    float CalJstickX     = 0;
    float CalJstickY     =   0;
    bool  spinning       = false;
    bool  spinningOld    = false;
    bool  moving         = false;
    int   skipPos; // wait befor starting position   controll
    int   freqA;
    int   freqB;


  protected:
    //PWM   *ptrpwm;
    int PinSleepSwitch;
    bool firstRun = true;

};
#endif
