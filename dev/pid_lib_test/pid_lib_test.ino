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
#include <PID_v1.h>
//#include <Stepper.h>

//math constants
#define earthConst 9.81

//-----------------------------------------------Pins----------------------------------------------------

//two motor driver boards on two set of pins
#define dirPin1 3
#define stepPin1 4
#define dirPin2 6
#define stepPin2 7

#define potiPin A0


const bool DEBUG = true;
//#########################################CONFIG/TUNING ZONE###############################################

// params for the steppies
const float maxSpeedLimit = (360) * 8;
const float maxAccelLimit = 3600 * 2;

// three dabloons for the three parameters of the P I D
float KP = 10;   //(P)roportional Tuning Parameter
float KI = 0;   //(I)ntegral Tuning Parameter 
float KD = 0.4; //(D)erivative Tuning Parameter

//this is the angle in degrees the robot should be standing at, measured perpendicular to ground.
//the bias is used to include the balancing point of the robot frame (determine this using the mpu test) 
float angleBias = -3.8;
float targetAngle = 0;


//##########################################################################################################


//---------------------------------------------stepper setup----------------------------------------------
// specs of the used stepper motor.
// if these differ from the setup, change to the stepper code is needed
// stepsPerRevolution = 200;
// stepTime = 1/8;


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
//find KI,KP and KP in the Config Zone (tm)

//error working vars for the PID algorithm
volatile float error, prevError = 0, errorSum = 0;

//some time working vars //TODO: currently unused?
unsigned long currTime, prevTime = 0, loopTime;
float sampleTime = 0.005;

//Define Variables we'll be connecting to
double setpoint, input, output;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint,2,5,1, DIRECT);


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

  //wait for the mpu to be foundon the I2C Bus
  while(mpu.wakeup() == false) {
    Serial.print(millis());
    Serial.println("\tConnection to GY521 failed");
    delay(1000);
  }
  Serial.println("GY521 FOUND IN THE BUS, STARTING PROGRAMM");
  delay(1000);

  //mpu setup
  mpu.setAccelSensitivity(0); //set to 4g
  mpu.setGyroSensitivity(0); //set to 500dps

  //sets the calibration values from the cali sketch
  mpu.axe = 0.0160767;
  mpu.aye = 0.0973950;
  mpu.aze = -0.9789429;
  mpu.gxe = 1.5724426;
  mpu.gye = 3.3177099;
  mpu.gze = 0.8232061;

  //mpu.setThrottle();

  //enable the ISR timer routine
  initISR();

    //initialize the variables we're linked to
  input = 0;
  setpoint = targetAngle;


  
  myPID.SetOutputLimits(-180, 180);

  myPID.SetControllerDirection(1);

  Serial.println("SETUP COMPLETED");

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
  int finValue = floatMap(analogValue, 0, 1023, minV, maxV);
  
  if (DEBUG) {
    Serial.print("Value: "); Serial.print(finValue); Serial.print(" "); 
  }

  return finValue;
}


void loop() {

  KP = readPoti(0,100);

  mpu.read(); //get new, fresh values
  
  //then, disect them into their components
  accZ = mpu.getAccelZ();
  accY = mpu.getAccelY();
  gyroX = mpu.getGyroX();
  angleX = mpu.getAngleX();
  //angleX = mpu.getRoll();
  //angleY = mpu.getAngleY(); //not needed atm
  //angleZ = mpu.getAngleZ();
  
  if(DEBUG){
    Serial.print("Angle: "); Serial.print(angleX); Serial.print(" ");
  }

  if(angleX > 90 || angleX < -90){
    
    //turn the PID off to prevent windup
    myPID.SetMode(MANUAL);
    output = 0;
  } else if(angleX < 10 && angleX > -10){

    //turn the PID on
     myPID.SetMode(AUTOMATIC);

  }

  //some debug print
  //Serial.println("");
  //Serial.print(accZ); Serial.print(" "); Serial.print(accY); Serial.print(" "); Serial.println(gyroX);
  //Serial.println(angleX);
  //Serial.print(angleX); Serial.print(" "); Serial.print(angleY); Serial.print(" "); Serial.print(angleZ); Serial.println(" ");

  //compute the needed motorpower using the PID

  //computePID();

  myPID.SetTunings(KP,KI,KD);
  input = angleX;
  myPID.Compute();
  motorPower = output;
  
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
