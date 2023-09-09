//TEST PROGRAMM FOR THE GY-521 MPU BREAKOUT BOARD
//
//Reads the z and y acceleration and calculates the resulting angle to ground eatch second
//
//
//
#include "GY521.h"
#include "math.h" 

#define earthConst 9.81

//stuff for the two steppers
#define dirPin1 2
#define stepPin1 3
#define dirPin2 4
#define stepPin2 5

#define stepsPerRevolution 200
#define stepTime 1

#include <AccelStepper.h>

AccelStepper rightStep(AccelStepper::DRIVER,stepPin1,dirPin1);
AccelStepper leftStep(AccelStepper::DRIVER,stepPin2,dirPin2);  

const int maxSpeedLimit = 2000.0;

//stuff for the sensing parts
GY521 mpu;

volatile float accY, accZ;
volatile float accAngle;

volatile float gyroX, gyroRate;
volatile float gyroAngle = 0;

volatile float angleX, angleY, angleZ;

unsigned long currTime, prevTime = 0, loopTime;

//values used for the PID
float KP = 40;
float KD = 0;// 0.05;
float KI = 0;//40;

float sampleTime = 0.005;
float targetAngle = 0;


//values used for the complimentary filter
float time = 0.9;
float alpha = time / (time + sampleTime);
float currentAngle, previousAngle;

volatile float error, prevError = 0, errorSum = 0;

volatile int motorPower;

void setup() {

  //because we do not want to explode the steppies
  rightStep.setMaxSpeed(maxSpeedLimit);
  leftStep.setMaxSpeed(maxSpeedLimit);

  Serial.begin(9600);
  Wire.begin();
  delay(100);
  while(mpu.wakeup() == false) {
    Serial.print(millis());
    Serial.println("\tConnection to GY521 failed");
    delay(1000);
  }
  Serial.println("GY521 FOUND IN THE BUS, STARTING PROGRAMM");
  delay(1000);
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

  //enable the PID timer routine
  //initPID();

  Serial.println("SETUP COMPLETED");
}
void loop() {

  mpu.read(); //get new, fresh values
  
  //then, disect them into their components
  accZ = mpu.getAccelZ();
  accY = mpu.getAccelY();
  gyroX = mpu.getGyroX();
  angleX = mpu.getAngleX();
  //angleY = mpu.getAngleY();
  //angleZ = mpu.getAngleZ();
  
  //Serial.println("");
  //Serial.print(accZ); Serial.print(" "); Serial.print(accY); Serial.print(" "); Serial.println(gyroX);
  //Serial.println(angleX);
  //Serial.print(angleX); Serial.print(" "); Serial.print(angleY); Serial.print(" "); Serial.print(angleZ); Serial.println(" ");

  computePID();


  //set motorpower
  motorPower = constrain(motorPower,-255,255); //again, damit die Stepper nicht platzen
  
  Serial.println(motorPower);
  setMotors(motorPower, motorPower);

  //delay(1000);
}

//some register magic taken from intstructables
//LINK: https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
//
//This enables the internal Timer1 to wake up the PID routine every 5ms
void initPID(){
  cli(); // disable global interrupts
  TCCR1A = 0; //set entire TCCR1A register to 0
  TCCR1B = 0; //same for TCCR1B

  //set compare match register to set sample time 5ms
  OCR1A = 9999;
  //turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable glonbal interrupts

} 


//the ISR will be called every 5ms
//ISR(TIMER1_COMPA_vect) {

//alternative: normal compute function
void computePID() {  
  //this is basically a makeshift PID-Controller

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;
  //gyroAngle = gyroX;

  //combining acc and gyro values using a complementary filter
  //
  //filter acts as a high pass on the gyro and a lowpass on the accel to
  //filter out drift and noise
  //currentAngle = alpha * (previousAngle + gyroAngle + sampleTime) + (1-alpha) * (accAngle);
  //currentAngle = 0.9934 * (previousAngle + gyroAngle) + 0.0066 * (accAngle);
  currentAngle = angleX;
  
  //calculation of error values for the PID
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300); //damit uns die Geschichte nicht um die Ohren fliegt

  //calculate output from P, I and D values
  motorPower = KP * (error)+ KI * (errorSum)*sampleTime - KD * (currentAngle-previousAngle)/sampleTime ;
  
  
  previousAngle = currentAngle; //save for next time
  //Serial.print(gyroAngle); Serial.print("  ");
  //Serial.print(accAngle); Serial.print("  ");
  //Serial.println(currentAngle);
  
}

void setMotors(float leftSpeed, float rightSpeed){
  //diff between the directions
  //negative speeds mean backwards
  if(leftSpeed >=0) {
    leftStep.setSpeed(leftSpeed);

  } else {
   leftStep.setSpeed(-leftSpeed);
  }

  //inverse the speeds at the second motor,
  //as its mounted mirrored
  if(rightSpeed >=0){
    rightStep.setSpeed(-rightSpeed);
  } else {
    rightStep.setSpeed(rightSpeed);
  }

  leftStep.runSpeed();
  rightStep.runSpeed();
}
