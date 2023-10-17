/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
    modified by : Cornelius Brütt in 2023

   Stepper Motor: Bipolar, 200 Steps/Rev, 4023mm, 12V, 1.5 A/Phase
  https://www.pololu.com/product/1200/faqs
   This NEMA 17-size hybrid stepping motor can be used as a unipolar or bipolar stepper   motor
  and has a 1.8 step angle (200 steps/revolution). Each phase draws 0.3   A at 12 V,
  allowing for a holding torque of 4.28 kg-cm (0.42Nm)

  Wheel diameter d= 70mm => Distance per Pulse Dpp = d * pi / 200 =  1,0996 mm
  Distance per Revolution =  pi * d = 219.911 mm
  Max 1000 Steps per Second = 5 Revolutions => 1099.55 mm Distance per Second

  Motion Control Modules  Stepper Motor Drivers  A4988 Stepper   Motor Driver Carriers
  https://www.pololu.com/product/1182

  The Motor Driver can be adjusted for different microstep resolutions. In the ARoPA MK-I, these will be fixed
  so no changing in software is possible.

  MS1 	MS2 	MS3 	Microstep Resolution
  Low 	Low 	Low 	Full step
  High 	Low 	Low 	Half step
  Low 	High 	Low 	Quarter step
  High 	High 	Low 	Eighth step
  High 	High 	High 	Sixteenth step
*/

#include   "Motor.h"
#include "Config.h"
/**********************************************************************/
//Motor::Motor(DuePWMmod*   ipwm, int iPinDir, int iPinStep,  char iMotorSide )
Motor::Motor( PWM* ipwm, int iPinDir, int iPinStep,  char iMotorSide )
/**********************************************************************/
{
  _PinStep   = iPinStep;
  _PinDir    = iPinDir;
  _MotorSide = iMotorSide;

  pinMode(_PinDir,   OUTPUT);
  pinMode(_PinStep,   OUTPUT);

  ptr =   (int) this;
  ptrpwm = ipwm;

  _Divisor = 8; //because we use 1/8 microstepping atm
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
  Serial.println("   init...");
  lastTime = millis();

}
/**********************************************************************/
void   Motor::SleepMode( )
/**********************************************************************/
{
  MotorMode = false;
}
/**********************************************************************/
void   Motor::RunMode( )
/**********************************************************************/
{
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
  if (!MotorMode)   {
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