/* Self balancing Robot via Stepper Motor with microstepping and Digital Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
    modified by : Cornelius Brütt in 2023

*/
#ifndef   Config_h
#define  Config_h
#include "Arduino.h"
#include "Filter.h"


/*   PIN Belegung
  for a complete setup overview, please refere to the fritzing sketch in the hrdw directory
  D3  Dir   Motor1
  D4  Step  Motor1
  
  D6  Dir   Motor2
  D7  Step  Motor2

  D9  ButtonInput Button1
  D10 ButtonInput Button2
  D11 LED1
  D12 LED2

  A0  Poti Input

  A4  SDA MPU
  A5  SCL MPU

*/

//we do not use any interrupts as we implement a polling strategy


const int  PinDirMotorA  = 3;
const int  PinDirMotorB  = 6;
const int  PinStepMotorA  = 4;
const int  PinStepMotorB   = 7;

const int  TuningPin      = A0; //Potentiometer


struct MpuYawPitchRoll   {
  float yaw, pitch, roll;
};


#endif
