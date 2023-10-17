/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de

   Stepper Motor: Unipolar/Bipolar, 200 Steps/Rev, 4248mm, 4V, 1.2 A/Phase
  https://www.pololu.com/product/1200/faqs
   This NEMA 17-size hybrid stepping motor can be used as a unipolar or bipolar stepper   motor
  and has a 1.8 step angle (200 steps/revolution). Each phase draws 1.2   A at 4 V,
  allowing for a holding torque of 3.2 kg-cm (44 oz-in).

  Wheel   Durchmesser 88mm = > Distance per Pulse Dpp = d * pi / 200 =  1,3816 mm
  Distance   per Revolution =  276,32 mm
  Max 1000 Steps per Second = 5 Revolutions => 13816   mm Distance

  Motion Control Modules  Stepper Motor Drivers  MP6500 Stepper   Motor Driver Carriers
  https://www.pololu.com/product/2966https://www.pololu.com/product/2966
   This breakout board for the MPS MP6500 microstepping bipolar stepper motor driver   has a
  pinout and interface that are very similar to that of our popular A4988   carriers,
  so it can be used as a drop-in replacement for those boards in many   applications.
  The MP6500 offers up to 1/8-step microstepping, operates from   4.5 V to 35 V,
  and can deliver up to approximately 1.5 A per phase continuously   without a heat sink
  or forced air flow (up to 2.5 A peak).

  MS1   MS2    Microstep Resolution
  Low   Low   Full step
  High  Low   Half (1/2) step
   Low   High  Quarter (1/4) step
  High  High  Eighth (1/8) step
*/

#include   "Motor.h"
#include "Config.h"
/**********************************************************************/
Motor::Motor(DuePWMmod*   ipwm, int iPinDir, int iPinStep,
             int iPinMs1, int iPinMs2, int iPinSleep,   char iMotorSide )
/**********************************************************************/
{
   _Ms1       =  iPinMs1;
  _Ms2       =  iPinMs2;
  _PinStep   = iPinStep;
   _PinDir    = iPinDir;
  _PinSleep  = iPinSleep;
  _MotorSide = iMotorSide;

   // The default SLEEP state prevents the driver from operating;
  // this pin   must be high to enable the driver
  pinMode(_PinSleep, OUTPUT);
  pinMode(_PinDir,   OUTPUT);
  pinMode(_Ms1, OUTPUT);
  pinMode(_Ms2, OUTPUT);

  ptr =   (int) this;
  ptrpwm = ipwm;
}
/**********************************************************************/
Motor*   Motor::getInstance()
/**********************************************************************/
{
   pMotor = this;
  return pMotor;
}
/**********************************************************************/
void   Motor::init ( )
/**********************************************************************/
{

   Serial.print("Motor ");
  Serial.print(ptr , HEX);
  Serial.print(" Side   ");
  Serial.print(_MotorSide);
  Serial.print(" iPinStep ");
  Serial.print(_PinStep);
   Serial.print(" iPinSleep ");
  Serial.print(_PinSleep);
  Serial.println("   init...");
  lastTime = millis();

}
/**********************************************************************/
void   Motor::SleepMode( )
/**********************************************************************/
{
   digitalWrite(_PinSleep, LOW);
  MotorMode = false;
}
/**********************************************************************/
void   Motor::RunMode( )
/**********************************************************************/
{
   digitalWrite(_PinSleep, HIGH);
  MotorMode = true;
}
/**********************************************************************/
void   Motor::toggleMode( )
/**********************************************************************/
{
   if ( MotorMode == false ) RunMode( );
  else  SleepMode();
}
/**********************************************************************/
float   Motor::Run(float Steps) {
  /**********************************************************************/

   //  Motor 20070C9C Side A iPinStep 6 iPinSleep 22  init...
  // Motor 20070CF4   Side B iPinStep 7 iPinSleep 24  init...

  if (!digitalRead(PinSleepSwitch))   {
    RunMode( );

    if (_MotorSide == rechterMotor) {
      if (Steps   >= 0 ) {
        digitalWrite(_PinDir, LOW);
        DirForward = true ;
       }
      else {
        digitalWrite(_PinDir, HIGH);
        DirForward   = false ;
      }
    } else
    {
      if (Steps >= 0 ) {
        digitalWrite(_PinDir,   HIGH);
        DirForward = true ;
      }
      else {
        digitalWrite(_PinDir,   LOW);
        DirForward = false ;
      }
    }

    if (_Divisor   > 0) {
      Steps = Steps * _Divisor; // convert into Microsteps
    }

     if ((abs(Steps) < 2.0))  Steps = 2.0;
    return Steps;
  }
  else   SleepMode( );
}
/**********************************************************************/
void   Motor::MsFull ( ) {
  digitalWrite(_Ms1, LOW);
  digitalWrite(_Ms2, LOW);
   _Divisor = 1;
}
void  Motor::MsHalf ( ) {
  digitalWrite(_Ms1, LOW);
   digitalWrite(_Ms2, HIGH);
  _Divisor = 2;
}
void  Motor::MsQuater ( )   {
  digitalWrite(_Ms1, HIGH);
  digitalWrite(_Ms2, LOW);
  _Divisor = 4;
}
void   Motor::MsMicrostep ( ) {
  digitalWrite(_Ms1, HIGH);
  digitalWrite(_Ms2,   HIGH);
  _Divisor = 8;
}
