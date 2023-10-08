/*
* This is the main code used for the balancing robot. Flash this to the Arduino Nano Memory
* 
* This software will determine the current angle of the robot and will try to correct it to the desiredAngle constant usind the PID controller
*/
//=============================================HEAD=======================================================
//used libraries
#include "math.h" 
#include <AccelStepper.h>
#include "KalmanMPU6050.h"

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
const int microStepConfiguration = 4 //1 for fullstep, 2 for halfstep, 4 for quarterstep...
const float maxSpeedLimit = 360 * microStepConfiguration;
const float maxAccelLimit = 8000;

// three dabloons for the three parameters of the P I D
float KP = 12;   //(P)roportional Tuning Parameter
float KI = 0;   //(I)ntegral Tuning Parameter 
float KD = 0.0; //(D)erivative Tuning Parameter

//this is the angle in degrees the robot should be standing at, measured perpendicular to ground.
//the bias is used to account for a slanted positioning of the MPU (determine this using the mpu test) 
float angleBias = -2.8;
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
  Wire.begin();
  delay(100); //delay to rule out some issues with Wire being slow

  //mpu setup
  IMU::init();
  IMU::read();

  //enable the ISR timer routine
  initISR();

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
  KP = readPoti(0,60);

  if (DEBUG) {
    Serial.print("KP: "); Serial.print(KP); Serial.print(" "); 
    Serial.print("KI: "); Serial.print(KI); Serial.print(" ");
    Serial.print("KD: "); Serial.print(KD); Serial.print(" ");
  }

  //Reads the data from the MPU...
  IMU::read();

   ää//...and processes it with the Kalman Filter
  theta= IMU::getRoll();

  //some debug print
  //Serial.println("");
  //Serial.print(accZ); Serial.print(" "); Serial.print(accY); Serial.print(" "); Serial.println(gyroX);
  //Serial.println(angleX);
  //Serial.print(angleX); Serial.print(" "); Serial.print(angleY); Serial.print(" "); Serial.print(angleZ); Serial.println(" ");

  //compute the needed motorpower using the PID

  computePID();
  
  if(DEBUG){
    //Serial.println(motorPower);
  }
  setMotors(motorPower, motorPower);

  //Slowing things down to prevent overflowing debug out
  //Comment this or the loop will be to slow!
  //delay(1000); 
}

/* some register magic taken from instructables
* LINK: https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
* 
* This enables the internal Timer1 to wake up the PID routine every 5ms

*  Calculations (for 500ms): 
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
  sei(); //enable glonbal interrupts

} 


//the ISR will be called every 5ms
ISR(TIMER1_COMPA_vect) {
  TCNT1  = 0; //reset the timer 
  // calling run() instead of runSpeed() tells the stepper to
  // accelerate to maxSpeed instead of using constant Speed
  leftStep.run();
  rightStep.run();

}
/*
* we currently do not depend on the ISR but instead call the PID function from the main loop func
* this is basically a makeshift PID-Controller
* This controller will do the following steps:
*
*     1. calculate the current angle twice using the accel data and the gyro data individually
*     2. combine these two angles using a complimentary filer, ruling out drift and noise
*     3. compute the PID error that specifies, how "bad" the current situation is
*     4. compute the needed motorPower using the PID algorithm with the magic KP,KI and KD numbers and errors
*/
void computePID() {  
  
  /*1. angle calculation*/

  //calc the current angle depending on the measured data
  //we currently use the already measured angle of the GY521 lib

  //accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  //gyroRate = map(gyroX, -32768, 32767, -250, 250); //mapping gyroX to int range 2^16 = 65 534
  //gyroAngle = (float)gyroRate*sampleTime;
  //gyroAngle = gyroX;

  /*2. combination of angles*/
  //combining acc and gyro values using a complementary filter
  //filter acts as a high pass on the gyro and a lowpass on the accel to filter out drift from gyro and noise from accel

  //currentAngle = alpha * (previousAngle + gyroAngle + sampleTime) + (1-alpha) * (accAngle);
  //currentAngle = 0.9934 * (previousAngle + gyroAngle) + 0.0066 * (accAngle);

  //as my comp filter is _really_ bad, use the comp filter from the GY521 lib and use measured angleX
  currentAngle = angleX;
  

  /*3. error calculation*/
  //calculation of error values for the PID
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300); //damit uns die Geschichte nicht um die Ohren fliegt


  /*4. finally, PID magic*/
  //calculate output from P, I and D values
  float output = KP * (error)+ KI * (errorSum)*sampleTime - KD * (currentAngle-previousAngle)/sampleTime ;
  motorPower = constrain(output, -360, 360); //a circle has 360 degrees, so constrain to one circle in each direction
  previousAngle = currentAngle; //save for next time

  //some debug outprint
  //Serial.print(gyroAngle); Serial.print("  ");
  //Serial.print(accAngle); Serial.print("  ");
  //Serial.println(currentAngle);
  
}

/*
* Function for controlling the left and right motor using
* a distance in degrees (°)
* positive values mean forward, negative mean backward rotation
*
*/
void setMotors(float leftDistance, float rightDistance){
  if(DEBUG) {
    Serial.println(leftDistance);
  }
  leftStep.move(leftDistance);
  rightStep.move(-rightDistance);

}
