#define mains 24
#define water 26
#define acid 28
#define base 30

int moistsensor = A0;
int phsensor = A2;

#include <Servo.h>
Servo arm;

#include <SoftwareSerial.h>
#define BT_TX 18
#define BT_RX 19

void setup() {
  pinMode(mains, OUTPUT);
  pinMode(water, OUTPUT);
  pinMode(acid, OUTPUT);
  pinMode(base, OUTPUT);
  pinMode(moistsensor, INPUT);
  pinMode(phsensor, INPUT);
  
  arm.attach(2);
  
  Serial.begin(9600);
  Serial1.begin(9600); // Use Serial1 for Mega's second serial port
}

void loop() {
  digitalWrite(mains, HIGH);
  digitalWrite(water, HIGH);
  digitalWrite(acid, HIGH);
  digitalWrite(base, HIGH);
  arm.write(0);

  if (Serial1.available()) {
    String signal = "";
    while (Serial1.available()) {
      char incomingByte = Serial1.read();
      signal += incomingByte;
      delay(100); // Wait for data to be received
    }
    
    // Print the received message to the Serial Monitor
    //Serial.print("Received from EV3: ");
    //Serial.println(signal.substring(7,11));

    if (signal.substring(7,11) == "go") {
      arm.write(100);
      digitalWrite(mains, LOW);
      delay(2000);
      
      int moistvalue = analogRead(moistsensor);
      int phvalue = analogRead(phsensor);

      Serial.println(phvalue);
      
      digitalWrite(mains, HIGH);
      arm.write(0);

      if (moistvalue > ) {
        if (phvalue > 900){
          delay(1500);
          digitalWrite(water, LOW);
          digitalWrite(acid, LOW);
          delay(1500);
          digitalWrite(water, HIGH);
          digitalWrite(acid, HIGH);
        }

        else if (phvalue < 800){
          delay(1500);
          digitalWrite(water, LOW);
          digitalWrite(base, LOW);
          delay(1500);
          digitalWrite(water, HIGH);
          digitalWrite(base, HIGH);
        }
      }

      else if (moistvalue < 700){
        if (phvalue > 900) {
          delay(1500);
          digitalWrite(acid, LOW);
          delay(1500);
          digitalWrite(acid, HIGH);
        }

        if (phvalue < 800) {
          delay(1500);
          digitalWrite(base, LOW);
          delay(1500);
          digitalWrite(base, HIGH);
        }
      }

    }
  }
}
