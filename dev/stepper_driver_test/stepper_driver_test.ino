/* Test programm for the stepper driver boards
*  This programm will test the integrity of the two stepper driver boards and motors
*
*  Note: The AccelStepper library has to be installed
*
*
*
*/

#define dirPin1 3
#define stepPin1 4
#define dirPin2 6
#define stepPin2 7

#define stepsPerRevolution 200
#define stepTime 1

#include <AccelStepper.h>

AccelStepper step1(AccelStepper::DRIVER,stepPin1,dirPin1);
AccelStepper step2(AccelStepper::DRIVER,stepPin2,dirPin2);  

const int maxSpeedLimit = 2000.0;

void setup() {
  step1.setMaxSpeed(maxSpeedLimit);
  step2.setMaxSpeed(maxSpeedLimit);
  
  step1.setSpeed(100);
  step2.setSpeed(100);

  //step1.runSpeed();
  //step2.runSpeed();

}

void loop() {
  
    
  step1.runSpeed();
  step2.runSpeed();
  //delay(2000);
  

  //step1.setSpeed(maxSpeedLimit/10.0);
  //step2.setSpeed(maxSpeedLimit/10.0);
  //step1.runSpeed();
  //step2.runSpeed();
  //delay(2000);
   
}
