/*
* This test programm checks the integrity of the built-in button panel. It includes two LEDs
* as well as two push-buttons including pull-up resistors. The power switch does not get tested as it isnt
* connected to the arduino in any way.
*
* The wiring of the lower button panel is taken from the fritzing sketches.
*
* Expected behaviour: 
* When Green BTN is pressed, Green LED should light up and print to Serial accordingly
* When Red BTN is pressed,   Red LED should light up   and print to Serial accordingly
* 
*
*/

#define RED_LED_PIN 9
#define GRN_LED_PIN 10

#define RED_BTN_PIN 11
#define GRN_BTN_PIN 12


void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GRN_LED_PIN, OUTPUT);
  
  pinMode(RED_BTN_PIN, INPUT);
  pinMode(GRN_BTN_PIN, INPUT);

  Serial.begin(9600);

}

void loop() {
  int red = digitalRead(RED_BTN_PIN);
  int green = digitalRead(GRN_BTN_PIN);

  //we read LOWs as we use pull-ups
  if(red == LOW){
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("RED PRESSED");
  } else {
    digitalWrite(RED_LED_PIN, LOW);
  }

    if(green == LOW){
    digitalWrite(GRN_LED_PIN, HIGH);
    Serial.println("GREEN PRESSED");
  } else {
    digitalWrite(GRN_LED_PIN, LOW);
  }

  delay(100); //debounce
}
