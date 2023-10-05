/* 
*   this will test the configuration of the onboard potentiometer
*/

#define POTI_PIN A0

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  
  Serial.begin(9600);

}

void loop() {
  int analogValue = analogRead(POTI_PIN);

  float voltage = floatMap(analogValue, 0, 1023, 0, 5);

  Serial.print("Analog: "); Serial.print(analogValue); Serial.print(", Voltage: "); Serial.println(voltage);
  delay(1000);

}
