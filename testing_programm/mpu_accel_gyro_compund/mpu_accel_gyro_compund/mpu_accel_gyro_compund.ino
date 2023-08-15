//TEST PROGRAMM FOR THE GY-521 MPU BREAKOUT BOARD
//
//Reads the z and y acceleration and calculates the resulting angle to ground eatch second
//
//
//
#include "GY521.h"
#include "math.h"

GY521 mpu;

float accY, accZ;
float accAngle;

float gyroX, gyroRate;
float gyroAngle = 0;

unsigned long currTime, prevTime = 0, loopTime;

//filter
float time = 0.75;
float alpha = time / (time + 0.0005);
float currentAngle, previousAngle;



void setup() {
  Serial.begin(9600);

  Wire.begin();

  mpu.setAccelSensitivity(1); //set to 4g
  mpu.setGyroSensitivity(1); //set to 500dps

  //sets the calibration values from the cali sketch
  mpu.axe = 0.0;
  mpu.aye = 0.0;
  mpu.aze = 0.0;

  mpu.gxe = 0.0;
  mpu.gye = 0.0;
  mpu.gze = 0.0;
}

void loop() {
  //keeping track of time
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

  mpu.read(); //get new, fresh values
  
  //then, disect them into their components
  accZ = mpu.getAccelZ();
  accY = mpu.getAccelY();

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  
  //NaN check
  if(isnan(accAngle));
  else
    //gets affected a lot by horizontal movement --> noise
    //Serial.println(accAngle);

  
  gyroX = mpu.getGyroX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  
  gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;

  //drifts after long periods of time --> drift
  //Serial.println(gyroAngle);


  //combining acc and gyro values using a complementary filter
  //
  //filter acts as a high pass on the gyro and a lowpass on the accel to
  //filter out drift and noise
  currentAngle = alpha * (previousAngle + gyroAngle) + (1-alpha) * (accAngle);
  previousAngle = currentAngle; //save for next time

  Serial.println(currentAngle);
  delay(5);
}
