/*
* This is the main code used for the balancing robot. Flash this to the Arduino Nano Memory
* 
* This software will determine the current angle of the robot and will try to correct it to the desiredAngle constant usind the PID controller
*
* Used Wheels: d=70mm -> U = pi * d = 220mm
* Rotation = 200 Steps/rev * MicroStep -> 1000 Steps/rev
* 
* MaxSpeed = 1000 Steps/Second = 1 rev/s = 220mm/s
* 
* Höhe Schwerpunkt h = 240mm
* Schräglage theta = 20 degrees
*
* Zu korrigierende Länge x = h * sin (theta) = 82.08mm
*
* --> Bei einer Schräglage von 20 Grad ist Aropa theoretsch im Stande, die notwendige Länge bei MaxSpeed = 1000 in 0.4s zu überwinden
*
*/
//=============================================HEAD=======================================================
//used libraries
#include "math.h" 
#include <AccelStepper.h>
#include "KalmanMPU6050.h"
#include <PID_v1.h>

//math constants
#define earthConst 9.81

//DEBUG MODE enables serial output
const bool DEBUG = true;

//-----------------------------------------------Pins----------------------------------------------------

//two motor driver boards on two set of pins
#define dirPin1 3
#define stepPin1 4
#define dirPin2 6
#define stepPin2 7

#define potiPin A0

//no pins for the MPU as it gets detected on the I2C Bus
//#########################################CONFIG/TUNING ZONE###############################################

// params for the steppies
const int microStepConfiguration = 4; //1 for fullstep, 2 for halfstep, 4 for quarterstep...
const float maxSpeedLimit = 360 * microStepConfiguration * 2;
const float maxAccelLimit = 8000;

// three dabloons for the three parameters of the P I D
float KP = 16;   //(P)roportional Tuning Parameter
float KI = 0;   //(I)ntegral Tuning Parameter 
float KD = 0.0; //(D)erivative Tuning Parameter

//this is the angle in degrees the robot should be standing at, measured perpendicular to ground.
//the bias is used to account for a slanted positioning of the MPU (determine this using the mpu test) 
float angleBias = -2.5;
float targetAngle = 0 + angleBias;


//##########################################################################################################


//---------------------------------------------stepper setup----------------------------------------------

//stepper Objects used in the code
AccelStepper rightStep(AccelStepper::DRIVER,stepPin2,dirPin2);
AccelStepper leftStep(AccelStepper::DRIVER,stepPin1,dirPin1);  

//working var that specifies the current motorPower
volatile int motorPower;

//---------------------------------------------sensor setup----------------------------------------------
//working vars for the accelerometer and gyro
volatile float theta;


//-----------------------------------------------PID setup----------------------------------------------
//find KI,KP and KP in the Config Zone (tm)

//error working vars for the PID algorithm
volatile float error, prevError = 0, errorSum = 0;

//some time working vars //TODO: currently unused?
unsigned long currTime, prevTime = 0, loopTime;
float sampleTime = 0.005;

//find the desired angle in the Config Zone (tm)

//values used for the complimentary filter
float time = 0.9;
float alpha = time / (time + sampleTime);
float currentAngle, previousAngle;


//PID vars
//Define Variables we'll be connecting to
double setpoint, input, output;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint,2,5,1, DIRECT);

//============================================= BODY =======================================================

void setup() {

  //because we do not want to explode the steppies
  rightStep.setMaxSpeed((float)maxSpeedLimit);
  leftStep.setMaxSpeed((float)maxSpeedLimit);

  //init the current Position
  leftStep.setCurrentPosition(0);
  rightStep.setCurrentPosition(0);

  //and determine the acceleration constant
  leftStep.setAcceleration(maxAccelLimit);
  rightStep.setAcceleration(maxAccelLimit);

  Serial.begin(9600);
  //Wire object is included the IMU lib atm
  //Wire.begin();
  //delay(100); //delay to rule out some issues with Wire being slow

  //mpu setup
  IMU::init();
  IMU::read();

  //enable the ISR timer routine
  initISR();

  //PID lib inits
  input = 0;
  setpoint = targetAngle;  
  myPID.SetOutputLimits(-360, 360);
  myPID.SetControllerDirection(1);

  Serial.println("SETUP COMPLETED");
  delay(1000);

}


/*
* util function that works like map, but for floats instead of integers
*/
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*
* Reads the onboard potentiometer and maps the analog value
* to a float between minV and maxV
*
*/
float readPoti(int minV, int maxV) {
  int analogValue = analogRead(potiPin);
  float finValue = floatMap(analogValue, 0, 1023, minV, maxV);

  return finValue;
}


void loop() {

  //read the tuning value of the poti
  //KP = readPoti(0,50);
  
  KD = readPoti(0,5);

  if (DEBUG) {
    Serial.print("KP:"); Serial.print(KP); Serial.print(" "); 
    Serial.print("KI:"); Serial.print(KI); Serial.print(" ");
    Serial.print("KD:"); Serial.print(KD); Serial.print(" ");
  }

  //Reads the data from the MPU...
  IMU::read();

  //...and processes it with the Kalman Filter
  theta = IMU::getRoll();

  //compute the needed motorpower using the PID
  //this sets the global motorPower to a new value
  //computePID();

  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(KP,KI,KD);
  input = theta;
  myPID.Compute();
  motorPower = output;

    if (DEBUG) {
    Serial.print("Current_Angle:");
    Serial.print(theta);
    Serial.print(",");
    Serial.print("Motor_Power:");
    Serial.println(motorPower);
  }


  //and set the Motors using the motorPower variable
  setMotors(motorPower, motorPower);

}

/* some register magic taken and from instructables and modified to my needs
*  LINK: https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
* 
*  This enables the internal Timer1 to interrupt the loop routine every 5ms
*  The time depends on the setting of the prescaler TCCR1B and the Counting 
*  register OCR1A
*  
*  Example Calculations (for 500ms): 
*  System clock 16 Mhz and Prescalar 256;
*  Timer 1 speed = 16Mhz/256 = 62.5 Khz    
*  Pulse time = 1/62.5 Khz =  16us  
*  Count up to = 500ms / 16us = 31250 (so this is the value the OCR register should have)  
*/
void initISR(){
  cli(); // disable global interrupts
  TCCR1A = 0; //set entire TCCR1A register to 0, resetting the timer value
  TCCR1B = 0; //same for TCCR1B
  //set compare match register to set sample time 5ms
  OCR1A = 100;
  //prescale: will divide the clock signal with 8, slightly slowing down our timer
  //TCCR1B |= (1 << CS11);
  TCCR1B |= B00000010;
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable global interrupts

} 

/*
* The Interrupt Service Routine (ISR) looks like a function
* that gets called in the specified timing intervall
*
* Our ISR will be called every 5ms to execute our run()
*/
ISR(TIMER1_COMPA_vect) {
  TCNT1  = 0; //reset the timer 
  // calling run() instead of runSpeed() tells the stepper to
  // accelerate to maxSpeed instead of using constant Speed
  leftStep.run();
  rightStep.run();

}
/*
* computes the new motorPower value using a PID-algorithm 
* We currently do not depend on the ISR for this call but instead call the PID function 
* from the main loop func ourselfs. This means that is isnt quite clear how much time
* lays inbetween calls, which will cause troubles later (foreshadowing)
*
* -> motorPower is a global variable because it needs to be when using the ISR
*
* This algorithm will do the following steps:
*
*     1. take the measured (and filtered!) angle from the mpu
*     2. compute the PID error that specifies, how "bad" the current situation is
*     3. compute the Proportional, Integral and Derivative terms of the PID with the magic KP,KI and KD numbers and errors
*     4. add the terms (and constrain the result!) the to calculate the needed motorPower (no return needed as motorPower is global)
*/
void computePID() {  

  /*1. take the angle */
  currentAngle = theta;

  if (DEBUG) {
    Serial.print("Current_Angle:");
    Serial.print(currentAngle);
    Serial.print(",");
  }

  /*2. error calculation*/
  //calculation of error values for the PID
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300); //damit uns die Geschichte nicht um die Ohren fliegt

  /*3. error calculation*/
  //calculate output from P, I and D values
  float pValue,iValue,dValue;
  pValue = KP * (error);
  iValue = KI * (errorSum)*sampleTime;
  dValue = KD * (currentAngle-previousAngle)/sampleTime;

  /*4. final steps*/
  float output = pValue + iValue - dValue;
  //a circle has 360 degrees, so constrain to one circle in each direction
  motorPower = constrain(output, -360, 360); 
  previousAngle = currentAngle; //save for next time

}

/*
* Function for controlling the left and right motor using a distance in degrees (°)
* positive values mean forward, negative mean backward rotation
*
*/
void setMotors(float leftDistance, float rightDistance){

  leftStep.move(leftDistance);
  rightStep.move(-rightDistance);

}

/*
* @deprecated
* This adds the gyro and accel angle using a comp filter
*
*/
//float addAngle(float gyro, float accel) {

  //calc the current angle depending on the measured data

  //accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  //gyroRate = map(gyroX, -32768, 32767, -250, 250); //mapping gyroX to int range 2^16 = 65 534
  //gyroAngle = (float)gyroRate*sampleTime;
  //gyroAngle = gyroX;

  /*2. combination of angles*/
  //combining acc and gyro values using a complementary filter
  //filter acts as a high pass on the gyro and a lowpass on the accel to filter out drift from gyro and noise from accel
  //float addedAngle = 0.0;
  //addedAngle = alpha * (previousAngle + gyroAngle + sampleTime) + (1-alpha) * (accAngle);

  //return addedAngle;
//}
