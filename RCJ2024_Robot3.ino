int ENA = 5; // Enable A for Motor 1 (PWM pin)
int IN1 = 6; // Motor 1 input 1
int IN2 = 7; // Motor 1 input 2

int ENB = 2; // Enable A for Motor 2 (PWM pin)
int IN3 = 3; // Motor 2 input 1
int IN4 = 4; // Motor 2 input 2

int trigger = A0; // Ultrasonic sensor trigger pin
int echo = A1; // Ultrasonic sensor echo pin
//int dc = 22; // Some other pin (use descriptive names for clarity)

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Minimum pulse length count (adjust as needed)
#define SERVOMAX  600 // Maximum pulse length count (adjust as needed)

void setup() {
  // put your setup code here, to run once:
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);

  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  //pinMode(dc, OUTPUT);

  Serial.begin(9600);
  mySerial.begin(9600);
  while (!huskylens.begin(mySerial)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  motorStop();
  motorForward();
  servo(0, 10);
  servo(1, 180);
  //digitalWrite(dc, HIGH);
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000; // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  pulselength /= 4096; // 12 bits of resolution
  pulse *= 1000;
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (ultrasonic() > 15) {
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if(!huskylens.available()) {
      Serial.println(F("No block or arrow appears on the screen!"));
      motorForward();}
    else
    {
        Serial.println(F("###########"));
        bool detect = false;
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);

            if(result.ID == 1){
              detect = true;
            }

        } 
        if (detect){
          motorForward();
          delay(600);
          motorStop();
          delay(500);
          motorL();
          delay(100);
          motorBackward();
          delay(2000);
          motorStop();
          delay(100);
          motorForward();
          delay(400);
          motorStop();
          delay(100);
          servo(0, 155);
          delay(1000);
          servo(1, 45);
          delay(500);
          servo(0, 10);
          delay(500);
          servo(1, 180); 
          delay(500);
          motorBackward();
          delay(1000);
          motorStop();
          delay(100);
          motorForward();
          delay(200);
          motorStop();
          delay(100);
          motorR();
          motorForward();
          delay(300);
        }
      }
  }
  
  
  if (ultrasonic() < 18) {
    motorStop();
    delay(200);
    motorL();
    motorBackward();
    delay(3000);
    while(ultrasonic() > 10){
      motorForward();
    }
    motorStop();
    delay(100);
    motorL();
    motorBackward();
    delay(5000);
    }
}

int ultrasonic() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  long duration = pulseIn(echo, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

void motorForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 150); // Adjust PWM value (0-255) for motor 1 speed
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150); // Adjust PWM value (0-255) for motor 2 speed
}

void motorBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150); // Adjust PWM value (0-255) for motor 1 speed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 150); // Adjust PWM value (0-255) for motor 2 speed
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); // Set PWM to 0 to stop motor 1
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0); // Set PWM to 0 to stop motor 2
}

void motorL() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
  delay(1500);

  motorStop();
}

void motorR() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200); // Adjust PWM value (0-255) for motor 1 speed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200); // Adjust PWM value (0-255) for motor 2 speed
  delay(1500);

  motorStop();
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}

uint16_t angleToPulse(int angle) {
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  
  return pulse;
}

void servo(uint8_t servoNum, int angle) {
  uint16_t pulse = angleToPulse(angle);
  pwm.setPWM(servoNum, 0, pulse);
}

