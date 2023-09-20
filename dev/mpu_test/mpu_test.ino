/* Test programm for the GY-521 MPU Breakout Board
*  
* This programm will automatically find the GY521 board on the I2C bus 
* and read out the x and y accel values. Going from these it will compute
* the current angle and print it to Serial.
*
* Note: This requires the GY521-lib from the repository to be installed
*
*
*
*/
#include "GY521.h"
#include "math.h"

//we change the default address as the AD0 pin isnt connected to anything
GY521 mpu(0x68);

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
