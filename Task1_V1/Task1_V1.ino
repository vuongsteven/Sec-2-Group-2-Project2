#include <Robojax_L298N_DC_motor.h>
#include "IRremote.h"

int receiver = 11;

// motor 2
int leftMotorForwardPin = 2;
int leftMotorBackwardPin = 4;
int leftMotorSpeedControl = 5;

// motor 1
int rightMotorForwardPin = 7;
int rightMotorBackwardPin = 8;
int rightMotorSpeedControl = 9;

int direction = 5; //0 - forward, 1 - backward, 2 - right, 3 - left

IRrecv irrecv(receiver); 
decode_results results;

void setup() {
  pinMode(leftMotorSpeedControl, OUTPUT);
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorSpeedControl, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);

//  digitalWrite(leftMotorSpeedControl, HIGH);
//  digitalWrite(rightMotorSpeedControl, HIGH);
  Serial.begin(115200);
  irrecv.enableIRIn();
}

void loop() {
  
  if(irrecv.decode(&results)) {
    switch(results.value) {
      case 0xFF18E7: goStraight(); Serial.print("straight"); break;
      case 0xFF10EF: rotateRight();  Serial.print("right"); break;
      case 0xFF5AA5: rotateLeft();  Serial.print("left"); break;     
      case 0xFF4AB5: goBackward();  Serial.print("backward"); break;
      case 0xFFFFFFFF: movePreviousDirection(); break;
      case 0xFF9867: goStraightMicro(); break;
      case 0xFF38C7: brake(); break;
    }
    irrecv.resume();
  }
}

void goStraight() {
  analogWrite(leftMotorSpeedControl, 150);
  analogWrite(rightMotorSpeedControl, 155);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(leftMotorForwardPin, HIGH);
  delay(800);
  brake();
  direction = 5;
}

void goStraightMicro() {
  analogWrite(leftMotorSpeedControl, 150);
  analogWrite(rightMotorSpeedControl, 255);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(leftMotorForwardPin, HIGH);
  delay(50);
  brake();
  direction = 6;
}

void rotateLeft() {
  analogWrite(leftMotorSpeedControl, 255);
  analogWrite(rightMotorSpeedControl, 255);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  delay(50);
  brake();
  direction = 3; 

}

void rotateRight() {
  analogWrite(leftMotorSpeedControl, 255);
  analogWrite(rightMotorSpeedControl, 255);
  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);
  delay(50);
  brake();
  direction = 2;

}

void goBackward() {
  analogWrite(leftMotorSpeedControl, 150);
  analogWrite(rightMotorSpeedControl, 150);
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);
  delay(1000);
  brake();
  direction = 1;

}

void brake() {
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
}

void turnRight90() {
  rotateRight();
  delay(240);
  brake();
}

void movePreviousDirection() {
  if(direction == 5) {
    goStraight();
  }else if(direction == 1) {
    goBackward();
  }else if(direction == 2) {
    rotateRight();
  }else if(direction == 3) {
    rotateLeft();
  }else if(direction == 6) {
    goStraightMicro();
  }else {
    brake();
   }
}
