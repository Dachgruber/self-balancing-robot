/* Test programm for the stepper driver boards
*  This programm will test the integrity of the two stepper driver boards and motors
*
*  Note: The AccelStepper library has to be installed
*
*
*
*/

#define dirPin1 3
#define stepPin1 4
#define dirPin2 6
#define stepPin2 7

#define stepsPerRevolution 200
#define stepTime 1

#include <AccelStepper.h>

AccelStepper step1(AccelStepper::DRIVER,stepPin1,dirPin1);
AccelStepper step2(AccelStepper::DRIVER,stepPin2,dirPin2);  

const int maxSpeedLimit = 32000.0;

void setup() {
  step1.setMaxSpeed(maxSpeedLimit);
  step2.setMaxSpeed(maxSpeedLimit);
  

  initPID();

  step1.setSpeed(-4800);
  step2.setSpeed(4800);

  //step1.runSpeed();
  //step2.runSpeed();

}

void loop() {
  
  //Serial.println("Test");

  //step2.setSpeed(-4800.00);
  //step1.runSpeed();
  //step2.runSpeed();
  //delay(2000);
  

  //step1.setSpeed(maxSpeedLimit/10.0);
  //step2.setSpeed(maxSpeedLimit/10.0);
  //step1.runSpeed();
  //step2.runSpeed();
  //delay(2000);
   
}

/* some register magic taken from instructables
* LINK: https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
* 
* This enables the internal Timer1 to wake up the PID routine every 5ms

*  Calculations:
*  System clock 16 Mhz and Prescalar 8;
*  Timer 1 speed = 16Mhz/8 = 2Mhz    
*  Pulse time = 1/2Mhz =  0.5us  
*  Count up to = 0.05ms / 0.5us = 100 (so this is the value the OCR register should have)  
*/
void initPID(){
  cli(); // disable global interrupts
  TCCR1A = 0; //set entire TCCR1A register to 0, resetting the timer value
  TCCR1B = 0; //same for TCCR1B

  //set compare match register to set sample time 5ms
  OCR1A = 100;
  //turn on CTC mode
  //TCCR1B |= (1 << WGM12);
  //Set CS11 bit for prescaling by 8
  //this will divide the clock signal with 8, speeding up  our timer
  TCCR1B |= B00000010; //sets prescaling to 8
  //TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); //enable glonbal interrupts

} 


//the ISR will be called every 5ms
ISR(TIMER1_COMPA_vect) {
  TCNT1  = 0; //reset the timer 
  Serial.print("ISR ");
  step1.runSpeed();
  step2.runSpeed();

}
