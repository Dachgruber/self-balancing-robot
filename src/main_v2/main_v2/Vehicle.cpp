/* Self balancing Robot via Stepper Motor with microstepping and Digital   Motion Processing
    written by : Rolf Kurth in 2019
    rolf.kurth@cron-consulting.de
    modified by : Cornelius Brütt in 2023
   The robot is controlled by PID controllers.
  The PID controllers ensures that   the robot remains upright.
  The PID controllers are implemented in the "PIDControl"   class.
  The standard PID algorithm was slightly modified after longer test series.   To the P-component the parameter Kx multiplied
  by the I-component was added.
*/
#include   "Vehicle.h"

/**********************************************************************/
Vehicle::Vehicle(PWM*  ipwm, Motor * MotorA, Motor * MotorB,  // Constructor
                 PidControl   * PIDController, PidControl * PIDControllerPos)
/**********************************************************************/
{
  pMotorA            = MotorA;
  pMotorB            = MotorB;
  pPIDController      = PIDController;
  pPIDControllerPos   = PIDControllerPos;
  ptrpwm             = ipwm;
  firstRun           = true;
}

/**********************************************************************/
void   Vehicle::Run(MpuYawPitchRoll YawPitchRoll , int &iPositionA, int &iPositionB)
/**********************************************************************/
{   /*
    For full calculations seen the motor.cpp file.
    Stepper Motor: Bipolar, 200 Steps/Rev => 1.8 degree per step
    Wheel Diameter 70mm = > Distance per Pulse Dpp = d * pi / 200 =  1,3816 mm
     Distance per Revolution =  276,32 mm
    Max Distance per Second = 1099.55 mm

    "iPositionA" in  microsteps
    8 microsteps   = 1 full Step
    1 microstepp = 0,125 full steps
    after division one change   in "PositionA" = 0.01 microstepp and 0,0125 full steps = 0,013745 mm
  */
   const int     tDelay = 10;

  PositionAB = ((float(iPositionA ) + float(iPositionB   )) / 100);
/*
  Serial.print(iPositionA );  //grau
  Serial.print(",");
   Serial.print(iPositionB );  //grau
  Serial.print(",");
  Serial.print(HoldPosition   );  //grau
  Serial.println(" ");
*/
  /********************* driverless   **************************************/
  if (firstRun) {
    firstRun = false;
    skipPos = - (2 * tDelay); // first time wait a little bit longer
    spinningOld = false;
    pMotorA->RunMode();
    pMotorB->RunMode();
    HoldPosition = PositionAB;
  }
  JStick.Xvalue = JStick.Xvalue - CalJstickX   ;
  JStick.Yvalue = JStick.Yvalue - CalJstickY;

  DeltaForward = float(JStick.Yvalue)   / 100.0 ;
  DeltaTurning = float(JStick.Xvalue ) / 4.0;

  if (!spinning) {
     if (++skipPos  >= tDelay) { // to delay the calls, Position Control should be 10 times lower than Motor Control
      skipPos = 0;
      // PID calculation   Delta Position
      // DeltaPos is necessary for the robot to keep its position clean after moving forward or backward
      DeltaPos =  pPIDControllerPos->calculate(HoldPosition,   PositionAB);
      // slowly adjust the hold position to the current position
       // so, DeltaPos is slowly reduced and the pPIDController
      // works   again without offset
      HoldPosition = (0.9 * HoldPosition) + ( 0.1 * PositionAB);
     }
  }

  // PID calculation of new steps
  StepsA = pPIDController->calculate(YawPitchRoll.pitch,-DeltaForward + DeltaPos );
  StepsB = StepsA;

  StepsA =  StepsA  + DeltaTurning;   // Moving right or left
  StepsB =  StepsB  - DeltaTurning;

  StepsA = pMotorA->Run(StepsA);  // run the motors
  StepsB = pMotorB->Run(StepsB);

  uint32_t freqA_abs = abs(StepsA); // only positive Steps
  uint32_t freqB_abs = abs(StepsB);  // direction via PinDir
  ptrpwm->setFreq2( freqA_abs, freqB_abs);

}

/**********************************************************************/
void   Vehicle::init()
/**********************************************************************/
{
   Serial.println("Vehicle Init Motor Pointer....");
  int ptr1 = (int) pMotorA;
  int ptr2 = (int) pMotorB;
  Serial.println(ptr1 , HEX);
  Serial.println(ptr2 , HEX);

  Serial.println("Vehicle Init PID Pointer....");
  int ptr3 = (int) pPIDController;
  int ptr4 = (int) pPIDControllerPos;
  Serial.println(ptr3 , HEX);
  Serial.println(ptr4 , HEX);

  pMotorA->init();
  pMotorB->init();

  pMotorA->MsMicrostep();  // set microstepping
  pMotorB->MsMicrostep();

  pMotorA->SleepMode();
  pMotorB->SleepMode();

}
/**********************************************************************/
void   Vehicle::Stop() {
  pMotorA->SleepMode( );
  pMotorB->SleepMode( );
}
