#define dirPin1 2
#define stepPin1 3
#define dirPin2 4
#define stepPin2 5

#define stepsPerRevolution 200
#define stepTime 1

#include <AccelStepper.h>

AccelStepper step1(AccelStepper::DRIVER,stepPin1,dirPin1);
AccelStepper step2(AccelStepper::DRIVER,stepPin2,dirPin2);  

const int maxSpeedLimit = 2000.0;

void setup() {
  step1.setMaxSpeed(maxSpeedLimit);
  step2.setMaxSpeed(maxSpeedLimit);
}

void loop() {
  

  step1.setSpeed(maxSpeedLimit/5.0);
  step2.setSpeed(maxSpeedLimit/5.0);
  step1.runSpeed();
  step2.runSpeed();
  //delay(2000);
  

  //step1.setSpeed(maxSpeedLimit/10.0);
  //step2.setSpeed(maxSpeedLimit/10.0);
  //step1.runSpeed();
  //step2.runSpeed();
  //delay(2000);
   
}
