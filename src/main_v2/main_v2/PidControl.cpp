/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    PID Controller
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
    modified by : Cornelius Brütt in 2023
*/
#include "PidControl.h"
#include   <Arduino.h>

/**********************************************************************/
PidControl::PidControl   (float iK, float iKp, float iKi, float iKd, float iKa, float iKx)
/**********************************************************************/
{   // Simple Constructor 1 with direct params association
  K  = iK;
  Kp = iKp;
  Kd = iKd;
  Ki = iKi;
  Ka = iKa;
  Kx = iKx;
  Last_error = 0;
  integrated_error = 0;
  first   = true;
}
/**********************************************************************/
PidControl::PidControl   (PidParameter Params)
/**********************************************************************/
{   // Constructor 2  for Motors, different PidParameter
  K  = Params.K;
  Kp = Params.Kp;
  Kd = Params.Kd;
  Ki = Params.Ki;
  Ka = Params.Ka;
  Kx = Params.Kx;
  Last_error = 0;
  integrated_error = 0;
  first = true;
}
/**********************************************************************/
PidControl::PidControl   (PidParameterPos Params)
/**********************************************************************/
{   // Constructor 3 for Distance, different PidParameter object type
  K  = Params.K;
  Kp = Params.Kp;
  Kd = Params.Kd;
  Ki = Params.Ki;
  Ka = Params.Ka;
  Kx = Params.Kx;
  Last_error = 0;
  integrated_error = 0;
  first = true;
}
/**********************************************************************/
PidControl*   PidControl::getInstance()
/**********************************************************************/
{
  pPID = this;
  return pPID;
}
/**********************************************************************/
void   PidControl::test ()
/**********************************************************************/
{
  Serial.print("PID Test ");
  ptr = (int) this;
  Serial.print("PIDptr   ");
  Serial.println(ptr , HEX);
}
/**********************************************************************/
float   PidControl::calculate (float iAngle, float isetPoint )
/**********************************************************************/
//   new calculation of Steps per Second // PID algorithm
{
  Now = micros();
   if (first) {
    first = false;
    Last_time = Now;
    integrated_error   = 0;
  }
  timeChange = (Now - Last_time)  ;
  timeChange = timeChange   / 1000.0;  // in millisec
  error = isetPoint -  iAngle;

  //differential term
  if ( timeChange   != 0) {
    dTerm =  1000.0 * Kd * (error - Last_error) /  timeChange  ;
   }

  //integrative term
  integrated_error = integrated_error  + ( error * timeChange );
   iTerm =   Ki * integrated_error / 1000.0;

  //proportional term
  pTerm =   Kp  * error + ( Ka   * integrated_error ); // modifying Kp

  // Compute PID Output in Steps per second
  eSpeed = K * (pTerm + iTerm + dTerm) ;

  //Remember something
  Last_time  = Now;
  Last_error = error;

  // digitalWrite(TestPIDtime,   !digitalRead(TestPIDtime)); // Toggle  Pin for reading the Frequenzy
  // eSpeed   = constrain (eSpeed , -500.0 , 500.0 ); // 10 Steps per Second because Microstep
   
 return eSpeed;  // Steps per Second
}

/**********************************************************************/
void   PidControl::reset ()
/**********************************************************************/
{
  integrated_error = 0.0;
  Last_error = 0.0;
}


/**********************************************************************/
void   PidControl::changePIDparams (PidParameter Params)
// changePIDparams = different   PidParameter !!!!
/**********************************************************************/
{
  K  = Params.K;
  Kp = Params.Kp;
  Kd = Params.Kd;
  Ki = Params.Ki;
  Ka = Params.Ka;
  Kx = Params.Kx;
}
/**********************************************************************/
void   PidControl::changePIDparams (PidParameterPos Params)
// changePIDparams = different   PidParameter !!!!
/**********************************************************************/
{   // different PidParameter !!!!
  K  = Params.K;
  Kp = Params.Kp;
  Kd = Params.Kd;
  Ki = Params.Ki;
  Ka = Params.Ka;
  Kx = Params.Kx;
}

/**********************************************************************/
float   PidControl::getSteps () {
  return eSpeed;
}
