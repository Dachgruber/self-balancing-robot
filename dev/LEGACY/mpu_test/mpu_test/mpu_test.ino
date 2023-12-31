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



void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);
  while(mpu.wakeup() == false) {
    Serial.print(millis());
    Serial.println("\tConnection to GY521 failed");
    delay(1000);
  }

  mpu.setAccelSensitivity(1); //set to 4g
  mpu.setGyroSensitivity(1); //set to 500dps
}

void loop() {
  mpu.read(); //get new, fresh values
  
  //then, disect them into their components
  accZ = mpu.getAccelZ();
  accY = mpu.getAccelY();

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  
  //NaN check
  if(isnan(accAngle));
  else
    Serial.println(accAngle);

  delay(1000);
}
