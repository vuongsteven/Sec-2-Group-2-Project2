//this code is a working demo for task 3

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
volatile float maxFrontDistance = 20.00;

NewPing sonar = NewPing(6, 12, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  
  pinMode(led, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);

delay(3000); // use the delay to place barney on the floor

  int count = 0;
  checkFrontDistance();
  while (count < 1) {
    goForward();
    checkFrontDistance();
    Serial.println(frontDistance);
    if (frontDistance < maxFrontDistance && frontDistance != 0) {
      Serial.println("Object detected. Turning right...");
      brake();
      count++;     
    }
  }

  goRight(); //goRight
  yaw = 0;
  int timer = 0;

  while (timer <= 150) {
    goForward();
    timer++;
  }
  brake();
  goLeft();
  yaw = 0;
  timer = 0;
  while (timer <= 350) {
    goForward();
    timer++;
  }
  brake();
  goLeft();
  yaw = 0;
  timer = 0;
  while (timer <= 150) {
    goForward();
    timer++;
  }
  brake();
  goRight();
  yaw = 0;
  timer = 0;
  while (timer <= 950) {
    goForward();
    timer++;
  }
  brake();
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
  analogWrite(ena, 95);
  digitalWrite(inc,HIGH);
  digitalWrite(ind,LOW);
  analogWrite(enb, 95); 
  digitalWrite(led, HIGH);
}

void left() {
  digitalWrite(ina, HIGH);
  digitalWrite(inb,LOW);
  analogWrite(ena, 95);
  digitalWrite(inc,LOW);
  digitalWrite(ind,HIGH);
  analogWrite(enb, 95); 
  digitalWrite(led, LOW);
}

void right() {
  digitalWrite(ina, LOW);
  digitalWrite(inb,HIGH);
  analogWrite(ena, 95);
  digitalWrite(inc,HIGH);
  digitalWrite(ind,LOW);
  analogWrite(enb, 95); 
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

void goLeft() {
  int counter = 0;
  while(1) {
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

    if ( yaw <= 92 && yaw >= 88 && counter < 101) { //WORKS FOR TURNING LEFT
          // digitalWrite(ina, HIGH);
          // digitalWrite(inb,HIGH);
          // analogWrite(ena, 0);
          // digitalWrite(inc,HIGH);
          // digitalWrite(ind,HIGH);
          // analogWrite(enb, 0); 
          // digitalWrite(led, HIGH);
          digitalWrite(ina, LOW);
          digitalWrite(inb,LOW);
          analogWrite(ena, 0);
          digitalWrite(inc,LOW);
          digitalWrite(ind,LOW);
          analogWrite(enb, 0); 
          digitalWrite(led, LOW);
          counter++;
          if(counter == 100)
            break;
    }
    else {
      if ( yaw < 90 ) { // left
          digitalWrite(ina, HIGH);
          digitalWrite(inb,LOW);
          analogWrite(ena, 85);
          digitalWrite(inc,LOW);
          digitalWrite(ind,HIGH);
          analogWrite(enb, 85); 
          digitalWrite(led, LOW);
    }

    else { // right
          digitalWrite(ina, LOW);
          digitalWrite(inb,HIGH);
          analogWrite(ena, 85);
          digitalWrite(inc,HIGH);
          digitalWrite(ind,LOW);
          analogWrite(enb, 85); 
          digitalWrite(led, LOW);
    }
    }
  }
}

void goRight() {
  int counter = 0;
  while(1) {
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

    if ( yaw >= -92 && yaw <= -88 && counter < 101) { //WORKS FOR TURNING LEFT
          // digitalWrite(ina, HIGH);
          // digitalWrite(inb,HIGH);
          // analogWrite(ena, 0);
          // digitalWrite(inc,HIGH);
          // digitalWrite(ind,HIGH);
          // analogWrite(enb, 0); 
          // digitalWrite(led, HIGH);
          digitalWrite(ina, LOW);
          digitalWrite(inb,LOW);
          analogWrite(ena, 0);
          digitalWrite(inc,LOW);
          digitalWrite(ind,LOW);
          analogWrite(enb, 0); 
          digitalWrite(led, LOW);
          counter++;
          if(counter == 100)
            break;
    }
    else {
      if ( yaw > -88 ) { // left
          digitalWrite(ina, LOW);
          digitalWrite(inb,HIGH);
          analogWrite(ena, 90);
          digitalWrite(inc,HIGH);
          digitalWrite(ind,LOW);
          analogWrite(enb, 90); 
          digitalWrite(led, LOW);
    }

    else { // right
          digitalWrite(ina, HIGH);
          digitalWrite(inb,LOW);
          analogWrite(ena, 90);
          digitalWrite(inc,LOW);
          digitalWrite(ind,HIGH);
          analogWrite(enb, 90); 
          digitalWrite(led, LOW);
    }
    }
  }
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
