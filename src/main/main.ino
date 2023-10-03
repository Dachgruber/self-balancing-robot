/*
* This is the main code used for the balancing robot. Flash this to the Arduino Nano Memory
* 
* This software will determine the current angle of the robot and will try to correct it to the desiredAngle constant usind the PID controller
*/
//=============================================HEAD=======================================================
//used libraries
#include "GY521.h"
#include "math.h" 
#include <AccelStepper.h>
//#include <Stepper.h>

//math constants
#define earthConst 9.81


const bool DEBUG = false;

//---------------------------------------------stepper setup----------------------------------------------
//two motor driver boards on two set of pins
#define dirPin1 3
#define stepPin1 4
#define dirPin2 6
#define stepPin2 7

//these depend on the used stepper motor. Use with care
//const int stepsPerRevolution = 200;
//const int stepTime = 1;
const int maxSpeedLimit = 2000.0 * 16;

//stepper Objects used in the code
AccelStepper rightStep(AccelStepper::DRIVER,stepPin2,dirPin2);
AccelStepper leftStep(AccelStepper::DRIVER,stepPin1,dirPin1);  



//working var that specifies the current motorPower
volatile int motorPower;

//---------------------------------------------sensor setup----------------------------------------------
//we do not need any pin specification as the MPU gets searched up on the I2C Bus

//working vars for the accelerometer and gyro
volatile float accY, accZ;
volatile float accAngle;

volatile float gyroX, gyroRate;
volatile float gyroAngle = 0;

volatile float angleX, angleY, angleZ;

//mpu object used in the code
//0x68 address as AD0 Pin is not connected atm
GY521 mpu(0x68);

//-----------------------------------------------PID setup----------------------------------------------
//three dabloons for the three parameters of the P I D
float KP = 10;
float KI = 40;//40;
float KD = 0.05;// 0.05;

//error working vars for the PID algorithm
volatile float error, prevError = 0, errorSum = 0;

//some time working vars //TODO: currently unused?
unsigned long currTime, prevTime = 0, loopTime;
float sampleTime = 0.005;

//this is the angle the robot should be standing at, measured perpendicular to ground. 0 for upright, +20 for 20 deg to the front etc
float targetAngle = 0;

//values used for the complimentary filter
float time = 0.9;
float alpha = time / (time + sampleTime);
float currentAngle, previousAngle;

//============================================= BODY =======================================================

void setup() {

  //because we do not want to explode the steppies
  rightStep.setMaxSpeed((float)maxSpeedLimit);
  leftStep.setMaxSpeed((float)maxSpeedLimit);

  Serial.begin(9600);
  Wire.begin();
  delay(100); //delay to rule out some issues with Wire being slow

  //wait for the mpu to be foundon the I2C Bus
  while(mpu.wakeup() == false) {
    Serial.print(millis());
    Serial.println("\tConnection to GY521 failed");
    delay(1000);
  }
  Serial.println("GY521 FOUND IN THE BUS, STARTING PROGRAMM");
  delay(1000);

  //mpu setup
  mpu.setAccelSensitivity(1); //set to 4g
  mpu.setGyroSensitivity(1); //set to 500dps

  //sets the calibration values from the cali sketch
  //mpu.axe = -0.0003540;
  //mpu.aye = -0.0029297;
  //mpu.aze = -0.9850341;
  
  //mpu.gxe = 1.4239695;
  //mpu.gye = 3.3001527;
  //mpu.gze = 0.8001526;

  //mpu.setThrottle();

  //enable the ISR timer routine
  initPID();

  Serial.println("SETUP COMPLETED");

}


void loop() {

  mpu.read(); //get new, fresh values
  
  //then, disect them into their components
  accZ = mpu.getAccelZ();
  accY = mpu.getAccelY();
  gyroX = mpu.getGyroX();
  angleX = mpu.getAngleX();
  //angleY = mpu.getAngleY(); //not needed atm
  //angleZ = mpu.getAngleZ();
  
  //some debug print
  //Serial.println("");
  //Serial.print(accZ); Serial.print(" "); Serial.print(accY); Serial.print(" "); Serial.println(gyroX);
  //Serial.println(angleX);
  //Serial.print(angleX); Serial.print(" "); Serial.print(angleY); Serial.print(" "); Serial.print(angleZ); Serial.println(" ");

  //compute the needed motorpower using the PID
  computePID();

  //and set motorpower
  motorPower = constrain(motorPower,-255*16,255*16); //constrain damit die Stepper nicht platzen
  
  //some debug print, this will slow down the motor significantly!
  if(DEBUG){
    Serial.println(motorPower);
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
void initPID(){
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
  //Serial.print("ISR ");
  leftStep.runSpeed();
  rightStep.runSpeed();

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
  motorPower = KP * (error)+ KI * (errorSum)*sampleTime - KD * (currentAngle-previousAngle)/sampleTime ;
  previousAngle = currentAngle; //save for next time

  //some debug outprint
  //Serial.print(gyroAngle); Serial.print("  ");
  //Serial.print(accAngle); Serial.print("  ");
  //Serial.println(currentAngle);
  
}

/*
* Function for controlling the left and right motor
* positive speeds mean forward, negative mean backward rotation
*
*/
void setMotors(float leftSpeed, float rightSpeed){

  //map the speed values to a more useful range
  const int RANGE = 255*16;
  float leftmSpeed = map(leftSpeed, -255, 255, -RANGE, RANGE);
  float rightmSpeed = map(rightSpeed, -255, 255, -RANGE, RANGE);

  Serial.println(leftmSpeed);

  //inverse the speeds at the second motor,
  //as its mounted mirrored
  leftStep.setSpeed(leftmSpeed);
  rightStep.setSpeed(- rightmSpeed);
  //leftStep.setSpeed(4000.00);
  //rightStep.setSpeed(4000.00)

  //run at new set speed
  leftStep.runSpeed();
  rightStep.runSpeed();

}
