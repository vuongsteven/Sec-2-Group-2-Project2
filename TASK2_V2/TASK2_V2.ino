//this code is a working demo for task 2

#include <Wire.h>
#include <MPU6050.h>
#include <NewPing.h>

#define SONAR_NUM 1
#define MAX_DISTANCE 200

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

int led = 13;
int ena = 5;
int enb = 9;
int ina = 2;
int inb = 4;
int inc = 7;
int ind = 8;

// Ultrasonic Sensor
volatile float frontDistance = 0;
volatile float maxFrontDistance = 10.00;

NewPing sonar = NewPing(6, 12, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibration, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  pinMode(led, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);

delay(5000); // use the delay to place barney on the floor

  int count = 0;
  checkFrontDistance();
  while (count < 3) {
    goForward();
    checkFrontDistance();
    Serial.println(frontDistance);
    if (frontDistance < maxFrontDistance && frontDistance != 0 && count == 0) {
      Serial.println("First object detected. Turning right...");
      brake();
      for (int i = 0; i < 25; i++) {
         rightMicro();
         delay(100);    
      }
      count++;     
    }
    checkFrontDistance();
    if (frontDistance < maxFrontDistance && frontDistance != 0 && count == 1) {
      Serial.println("Second object detected. Turning left...");
      brake();
      for (int i = 0; i < 25; i++) {
         leftMicro();
         delay(100);    
      }
      count++;
    }

  }
}

void loop() {
  //goForward();
}

void checkFrontDistance() {
  frontDistance = sonar.ping_cm();
}

void forward() {
  digitalWrite(ina, HIGH);
  digitalWrite(inb, LOW);
  analogWrite(ena, 90);
  digitalWrite(inc,HIGH);
  digitalWrite(ind,LOW);
  analogWrite(enb, 90); 
  digitalWrite(led, HIGH);
}

void left() {
  digitalWrite(ina, HIGH);
  digitalWrite(inb,LOW);
  analogWrite(ena, 90);
  digitalWrite(inc,LOW);
  digitalWrite(ind,HIGH);
  analogWrite(enb, 90); 
  digitalWrite(led, LOW);
}

void right() {
  digitalWrite(ina, LOW);
  digitalWrite(inb,HIGH);
  analogWrite(ena, 90);
  digitalWrite(inc,HIGH);
  digitalWrite(ind,LOW);
  analogWrite(enb, 90); 
  digitalWrite(led, LOW);
}

void leftMicro() {
  digitalWrite(ina, HIGH);
  digitalWrite(inb,LOW);
  analogWrite(ena, 255);
  digitalWrite(inc,LOW);
  digitalWrite(ind,HIGH);
  analogWrite(enb, 255); 
  digitalWrite(led, LOW);
  delay(20);
  brake();
}

void rightMicro() {
  digitalWrite(ina, LOW);
  digitalWrite(inb,HIGH);
  analogWrite(ena, 255);
  digitalWrite(inc,HIGH);
  digitalWrite(ind,LOW);
  analogWrite(enb, 255); 
  digitalWrite(led, LOW);
  delay(20);
  brake();
}

void goForward() {
  timer = millis();

  // Read normalized values
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
}

void goRight() {
  Serial.println("goRight");
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  // Serial.print(" Pitch = ");
  // Serial.print(pitch);
  // Serial.print(" Roll = ");
  // Serial.print(roll);  
  // Serial.print(" Yaw = ");
  // Serial.println(yaw);

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));

  if (yaw <= -44 && yaw >= -46) {
    Serial.print("The robot turned right");
    brake();
  }else{
    if(yaw > -43) {
      right();
    }
  }

  // if (yaw >= -92) {
  //   Serial.print("The robot turned right");
  //   right();
  // }else {
  //   brake();
  // }
}

void brake() {
  digitalWrite(ina, LOW);
  digitalWrite(inb, LOW);
  analogWrite(ena, 0);
  digitalWrite(inc, LOW);
  digitalWrite(ind, LOW);
  analogWrite(enb, 0); 
  digitalWrite(led, LOW);
}