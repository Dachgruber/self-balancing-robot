#define dirPin1 2
#define stepPin1 3
#define dirPin2 4
#define stepPin2 5

#define stepsPerRevolution 200

#define stepTime 1
void setup() {

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

}

void loop() {

    //set spinning direction
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    for(int i = 0; i< stepsPerRevolution;i++){
      //one loop block equals one step
      digitalWrite(stepPin1, HIGH);
      digitalWrite(stepPin2, HIGH);
      delay(stepTime);
      digitalWrite(stepPin1, LOW);
      digitalWrite(stepPin2, LOW);
      delay(stepTime);
    }

    delay(1000);

    //set spinning direction
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    for(int i = 0; i< stepsPerRevolution;i++){
      //one loop block equals one step
      digitalWrite(stepPin1, HIGH);
      digitalWrite(stepPin2, HIGH);
      delay(stepTime);
      digitalWrite(stepPin1, LOW);
      digitalWrite(stepPin2, LOW);
      delay(stepTime);
    }

    delay(1000);
}
