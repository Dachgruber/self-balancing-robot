/* Test programm for the GY-521 MPU Breakout Board
*  
* This programm will automatically find the GY521 board on the I2C bus 
* and read out the x and y accel values as well as the x angle computed by the lib. 
* Going from these it will compute the current angle using the accel values and 
* print either the accAngle or the lib angle to Serial.
*
* 
*
* Note: This requires the GY521-lib from the repository to be installed
*
*
*
*/
#include "math.h"
#include "KalmanMPU6050.h"

const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup()
{
  Serial.begin(115200);

  IMU::init();
  IMU::read();

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  /* Reads the data from the MPU and processes it with the Kalman Filter */
  IMU::read();


  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  if (millis() > refresh_time) {
  Serial.print(IMU::getRoll());
  Serial.print(",");
  Serial.println(IMU::getPitch());
    
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

  //delay(1000);
}
