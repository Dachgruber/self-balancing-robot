/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
     Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
     by Jeff Rowberg <jeff@rowberg.net>

    MPU-6050 Accelerometer + Gyro

     The MPU-6050 sensor contains a MEMS accelerometer and a MEMS gyro in a single   chip. 
    It is very accurate, as it contains 16-bits analog to digital conversion   hardware for each channel. 
    Therefor it captures the x, y, and z channel   at the same time. The sensor uses the I2C -bus to interface with the Arduino.

     I used Digital Motion Processing with the MPU-6050 sensor, to do fast calculations   directly on the chip. 
    This reduces the load for the Arduino.  

    https://playground.arduino.cc/Main/MPU-6050

     Because of the orientation of my board, I used yaw/pitch/roll angles (in degrees)   calculated from the quaternions coming from the FIFO. 
    By reading Euler angle   I got problems with Gimbal lock.    
*/

/*    0x02,   0x16,   0x02,   0x00,   0x07                // D_0_22 inv_set_fifo_rate

    // This very last 0x01   WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01   is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
     // DMP output frequency is calculated easily using this equation: (200Hz / (1   + value))

    // It is important to make sure the host processor can keep   up with reading and processing
    // the FIFO output at the desired rate. Handling   FIFO overflow cleanly is also a good idea.
*/
// ------------------------------------------------------------------------
//   orientation/motion vars
// ------------------------------------------------------------------------
Quaternion   q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         //   [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x,   y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;     // [x, y, z]            world-frame accel sensor measurements
VectorFloat   gravity;    // [x, y, z]            gravity vector
float euler[3];         //   [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch,   roll]   yaw/pitch/roll container and gravity vector
// ------------------------------------------------------------------------
//   MPU control/status vars
// ------------------------------------------------------------------------
bool      dmpReady = false;  // set true if DMP init was successful
uint8_t  mpuIntStatus;       // holds actual interrupt status byte from MPU
uint8_t  devStatus;         //   return status after each device operation (0 = success, !0 = error)
uint16_t   packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t   fifoCount;         // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64];     // FIFO storage buffer



// --------------------- Sensor aquisition   -------------------------
MpuYawPitchRoll ReadMpuRunRobot()
// ---------------------   Sensor aquisition  -------------------------
// wait for MPU interrupt or extra   packet(s) available
// --------------------------------------------------------------------
{
   if (mpuInterrupt or fifoCount >= packetSize) {
    if  (mpuInterrupt) mpuInterrupt   = false;  // reset interrupt flag
    digitalWrite(LED_PIN, HIGH); // blink LED   to indicate activity
    //    Angle = ReadMpuRunRobot6050() - CalibrationAngle;
     YawPitchRoll = ReadMpuRunRobot6050();
    YawPitchRoll.pitch  +=  Calibration;
     // blinkState = !blinkState;
    digitalWrite(LED_PIN, LOW);
  }
  return   YawPitchRoll ;
}
// --------------------------------------------------------------------   -
MpuYawPitchRoll ReadMpuRunRobot6050()
// --------------------------------------------------------------------   -
{
  static float pitch_old;
  static float yaw_old;
  static float   yaw_tmp;
  static float yaw_delta;
  //  static float pitch;

  mpuIntStatus   = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
   // check for overflow (this should never happen unless our code is too inefficient)
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue   cleanly
    //    mpu.setDMPEnabled(false);
    mpu.resetFIFO();
    FifoOverflowCnt   ++;
    fifoCount = 0;
    YawPitchRoll.pitch = pitch_old;
    return YawPitchRoll;

   }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
   // Register 58  lnterrupt Status INT_STATUS
  // MPU-6500 Register Map and   Descriptions Revision  2.1
  // Bit [1] DMP_INT This bit automatically sets to   1 when the DMP interrupt has been generated.
  // Bit [0] RAW_DATA_RDY_INT1 Sensor   Register Raw Data sensors are updated and Ready to be read.
  if ((mpuIntStatus)   & 0x02 || (mpuIntStatus & 0x01)) {
    // wait for correct available data length,   should be a VERY short wait
    while (fifoCount < packetSize)  fifoCount = mpu.getFIFOCount();

     while (fifoCount > 0) {
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer,   packetSize);

      // track FIFO count here in case there is > 1 packet available
       // (this lets us immediately read more without waiting for an interrupt)
       fifoCount = mpu.getFIFOCount();
      //      fifoCount -= packetSize;
     }
    // the yaw/pitch/roll angles (in degrees) calculated from the quaternions   coming
    // from the FIFO. Note this also requires gravity vector calculations.
     // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
    //   more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

    mpu.dmpGetQuaternion(&q,   fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    // mpu.dmpGetEuler(euler,   &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    YawPitchRoll.pitch   = -(ypr[1] * 180 / M_PI); //pitch

    yaw_tmp  = (abs(ypr[0] * 180 / M_PI));
     yaw_delta = yaw_tmp - yaw_old;
    yaw_old = yaw_tmp;
    YawPitchRoll.yaw   += yaw_delta;

    // actual quaternion components in a [w, x, y, z]
     // YawPitchRoll.pitch = (q.y) * 180;
    // YawPitchRoll.yaw = (q.z );
     // YawPitchRoll.yaw =  mapfloat(YawPitchRoll.yaw , -1.0, 1.0, 0.0, 360.0);
   }
  pitch_old = YawPitchRoll.pitch ;
  return  YawPitchRoll ;
}
//   --------------------------------------------------------------
void MpuInit()
//   --------------------------------------------------------------
// MPU6050_6Axis_MotionApps20.h
//   0x02,   0x16,   0x02,   0x00, 0x09  => 50msec 20 Hz
// This very last 0x01 WAS   a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
// 0x01 is 100Hz.   Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
// DMP   output frequency is calculated easily using this equation: (200Hz / (1 + value))
//   It is important to make sure the host processor can keep up with reading and processing
//   the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good   idea.

{
  // after Reset of Arduino there is no Reset of MPU
  pinMode(MpuInterruptPin,   INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050   connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing   DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets   here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
   mpu.setZGyroOffset(-84);
  mpu.setZAccelOffset(1788); //

  // make sure   it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP,   now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

     // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt   detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(MpuInterruptPin),   dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set   our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP   ready! Waiting for first interrupt..."));
    dmpReady = true;

    //   get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
     //    mpu.resetFIFO();  // after Reset importand

  } else {
    //   ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration   updates failed
    // (if it's going to break, usually the code will be 1)
     Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
     Serial.println(F(")"));
    lcd.clear();
    lcd.setCursor(0, 0);
     lcd.print( "Error MPU6050 "  );
    do  {} while ( 1 == 1);
  }
}

float   mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
