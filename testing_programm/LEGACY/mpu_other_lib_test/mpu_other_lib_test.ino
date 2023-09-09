//TEST PROGRAMM FOR THE GY-521 MPU BREAKOUT BOARD
//
//Reads the z and y acceleration and calculates the resulting angle to ground eatch second
//
//
//
//#include "GY521.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include "math.h" 

#include <AccelStepper.h>

#define earthConst 9.81

//stuff for the two steppers
#define dirPin1 4
#define stepPin1 5
#define dirPin2 6
#define stepPin2 7

#define interrupt_pin 2

#define stepsPerRevolution 200
#define stepTime 1


AccelStepper rightStep(AccelStepper::DRIVER,stepPin1,dirPin1);
AccelStepper leftStep(AccelStepper::DRIVER,stepPin2,dirPin2);  

const int maxSpeedLimit = 2000.0;

//stuff for the sensing parts
MPU6050 mpu(0x69);

int16_t accY, accZ;
volatile float accAngle;

int16_t gyroX;
volatile int gyroRate;
volatile float gyroAngle;

//currently unused
//volatile float angleX, angleY, angleZ;


//values used for the PID
#define KP 40
#define KD 0// 0.05;
#define KI 0//40;

#define sampleTime 0.005
#define targetAngle 0


//values used for the complimentary filter
float time = 0.9;
float alpha = time / (time + sampleTime);
volatile float currentAngle, previousAngle = 0;

volatile float error, prevError = 0, errorSum = 0;

volatile int motorPower;

void setup() {

  //because we do not want to explode the steppies
  rightStep.setMaxSpeed(maxSpeedLimit);
  leftStep.setMaxSpeed(maxSpeedLimit);


  Serial.begin(9600);
  //Wire.begin();
  delay(100);
  mpu.initialize();
  pinMode(interrupt_pin, INPUT);
  Serial.println("GY521 FOUND IN THE BUS, STARTING PROGRAMM");
  delay(1000);
  //mpu.setAccelSensitivity(1); //set to 4g
  //mpu.setGyroSensitivity(1); //set to 500dps

  //sets the calibration values from the cali sketch
  //mpu.axe = -0.0003540;
  //mpu.aye = -0.0029297;
  //mpu.aze = -0.9850341;
  
  //mpu.gxe = 1.4239695;
  //mpu.gye = 3.3001527;
  //mpu.gze = 0.8001526;

  //mpu.setThrottle();

  //enable the PID timer routine
  initPID();

  Serial.println("SETUP COMPLETED");
}
void loop() {

  //mpu.read(); //get new, fresh values
  
  //then, disect them into their components
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
  gyroX = mpu.getRotationX();


 
  //angleX = mpu.getAngleX();
  //angleY = mpu.getAngleY();
  //angleZ = mpu.getAngleZ();

  Serial.print(accZ); Serial.print(" "); Serial.print(accY); Serial.print(" "); Serial.println(gyroX);

  //Serial.print(angleX); Serial.print(" "); Serial.print(angleY); Serial.print(" "); Serial.print(angleZ); Serial.println(" ");

  //set motorpower
  motorPower = constrain(motorPower,-255,255); //again, damit die Stepper nicht platzen
  setMotors(motorPower, motorPower);

  //delay(100);
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
ISR(TIMER1_COMPA_vect) {
  //this is basically a makeshift PID-Controller

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroX*sampleTime;
  //gyroAngle = gyroX;

  //combining acc and gyro values using a complementary filter
  //
  //filter acts as a high pass on the gyro and a lowpass on the accel to
  //filter out drift and noise
  currentAngle = alpha * (previousAngle + gyroAngle) + (1-alpha) * (accAngle);
  //currentAngle = 0.9934 * (previousAngle + gyroAngle) + 0.0066 * (accAngle);

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
