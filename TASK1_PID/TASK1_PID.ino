#include <Wire.h>
#include <MPU6050.h>
#include "IRremote.h"

int receiver = 11;
IRrecv irrecv(receiver); 
decode_results results;

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

int ena, leftMotorSpeedControl = 5;
int enb, rightMotorSpeedControl = 9;
int ina, leftMotorForwardPin = 2;
int inb, leftMotorBackwardPin = 4;
int inc, rightMotorForwardPin = 7;
int ind, rightMotorBackwardPin = 8;

int direction = 5; //5 - forward, 1 - backward, 2 - right, 3 - left, 6 - microstep forward

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);

delay(3000); // use the delay to place robot on the floor
  
}

void loop() {
  if(irrecv.decode(&results)) {
    switch(results.value) {
      case 0xFF18E7: yaw = 0; goForward(); Serial.print("straight"); break;
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

void forward() {
  digitalWrite(ina, HIGH);
  digitalWrite(inb, LOW);
  analogWrite(ena, 150); //95
  digitalWrite(inc,HIGH);
  digitalWrite(ind,LOW);
  analogWrite(enb, 150); 
}

void left() {
  digitalWrite(ina, HIGH);
  digitalWrite(inb,LOW);
  analogWrite(ena, 110);
  digitalWrite(inc,LOW);
  digitalWrite(ind,HIGH);
  analogWrite(enb, 110); 
}

void right() {
  digitalWrite(ina, LOW);
  digitalWrite(inb,HIGH);
  analogWrite(ena, 110);
  digitalWrite(inc,HIGH);
  digitalWrite(ind,LOW);
  analogWrite(enb, 110); 
}

void brake() {
  digitalWrite(ina, LOW);
  digitalWrite(inb, LOW);
  analogWrite(ena, 0);
  digitalWrite(inc, LOW);
  digitalWrite(ind, LOW);
  analogWrite(enb, 0); 
}

void goForward() {
  int counter = 0;
  while(counter < 150) {
    timer = millis();

    Vector norm = mpu.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;

    // Output raw
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.print(roll);  
    Serial.print(" Yaw = ");
    Serial.println(yaw);

    // Wait to full timeStep period
    delay((timeStep*1000) - (millis() - timer));

    if ( yaw >= -2 && yaw <= 2) {
      forward();
    }
    else {
      if ( yaw < -2 ) { //left
        left();
      }

      else {
        right();
      }
    }
    counter++;
  }
  direction = 5;
  brake();
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

void movePreviousDirection() {
  if(direction == 5) {
    goForward();
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
