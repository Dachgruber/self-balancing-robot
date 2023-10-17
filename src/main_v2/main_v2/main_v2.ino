/*#########################################################################################################################
* This is the second iteration of the balancing robot software, intended for the MK-I ARoPa-Bot
*
*
*
*
*
*
*
*
* ######################################################################################################################### */
// ------------------------------------------------------------------------
//  create MPU-6050 objects
// ------------------------------------------------------------------------
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if  I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050  mpu; // create object mpu
// ------------------------------------------------------------------------
//  create PID Controller
// ------------------------------------------------------------------------
#include  "PidControl.h"
PidParameter PidParams;
PidParameterPos PidParamsPos;

PidControl  PidControler(PidParams);
PidControl PidControlerPos(PidParamsPos);


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
