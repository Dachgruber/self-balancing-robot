/*
* This test programm checks the integrity of the built-in button panel. It includes two LEDs
* as well as two push-buttons including pull-up resistors. The power switch does not get tested as it isnt
* connected to the arduino in any way.
*
* The wiring of the lower button panel is taken from the fritzing sketches.
*
*/

#define RED_LED_PIN 2
#define GRN_LED_PIN 3

#define RED_BTN_PIN 4
#define GRN_BTN_PIN 5


void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GRN_LED_PIN, OUTPUT);
  
  pinMode(RED_BTN_PIN, INPUT);
  pinMode(GRN_BTN_PIN, INPUT);

}

void loop() {
  int red = digitalRead(RED_BTN_PIN);
  int green = digitalRead(GRN_BTN_PIN);

  //we read LOWs as we use pull-ups
  if(red == LOW){
    digitalWrite(RED_LED_PIN, HIGH);
  } else {
    digitalWrite(RED_LED_PIN, LOW);
  }

    if(green == LOW){
    digitalWrite(GRN_LED_PIN, HIGH);
  } else {
    digitalWrite(GRN_LED_PIN, LOW);
  }

  delay(100); //debounce
}
