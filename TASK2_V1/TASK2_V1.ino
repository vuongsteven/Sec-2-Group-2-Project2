#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

// right motor 
int rightMotorForwardPin = 2;
int rightMotorBackwardPin = 4;
int rightMotorSpeedControl = 5;

// left motor
int leftMotorForwardPin = 7;
int leftMotorBackwardPin = 8;
int leftMotorSpeedControl = 9;

int direction = 5; //0 - forward, 1 - backward, 2 - right, 3 - left

volatile float frontDistance = 0;
volatile float leftDistance = 0;
volatile float rightDistance = 0;

volatile float maxFrontDistance = 10.00;
volatile float maxLeftDistance = 20.00;
volatile float maxRightDistance = 20.00;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(6, 10, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(6, 12, MAX_DISTANCE), 
  NewPing(6, 13, MAX_DISTANCE)
};

void setup() {
  pinMode(leftMotorSpeedControl, OUTPUT);
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorSpeedControl, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);

  Serial.begin(115200);

  int count = 0;
  checkFrontDistance();
  while (count < 3) {
    goStraight();
    checkFrontDistance();
    if (frontDistance < maxFrontDistance && count == 0) {
      Serial.println("First object detected. Turning right...");
       for (int i = 0; i < 30; i++) {
         rotateRight();
         delay(100);    
       }  
      delay(2500);
      for (int i = 0; i < 30; i++) {
        goStraightMicro();
        delay(100);
      }
      count++;
    }
    
    checkFrontDistance();
    if (frontDistance < maxFrontDistance && count == 1) {
      Serial.println("Second object detected. Turning left...");
      for (int i = 0; i < 30; i++) {
         rotateLeft();
         delay(100);    
       }
      count++;
      delay(3000);
      for (int i = 0; i < 30; i++) {
        goStraightMicro();
        delay(100);
      }
    }
  }
  // if (frontDistance < maxFrontDistance && count == 0) {
  //   Serial.println("First object detected. Turning right...");
  //   for (int i = 0; i < 10; i++) {
  //     rotateRight();
  //     delay(100);
  //     Serial.println("Count: ");
  //     Serial.println(count);
  //   }
  //   count++;
  // } else if (frontDistance < maxFrontDistance && count == 1) {
  //   Serial.println("Second object detected. Turning left...");
  //   for (int i = 0; i < 10; i++) {
  //     rotateLeft();
  //     delay(100);
  //     Serial.println("Count: ");
  //     Serial.println(count);
  //   }
  //   count++;
  // } else {
  //   goStraight();
  // }
}

void loop() {
  //for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    //delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    //Serial.print(i);
    //Serial.print("=");
    //Serial.print(sonar[i].ping_cm());
    //Serial.print("cm ");
  //}
  //Serial.println();
}

// FUNCTIONS //

void goStraight() {
  analogWrite(leftMotorSpeedControl, 120);
  analogWrite(rightMotorSpeedControl, 120);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(leftMotorForwardPin, HIGH);
  //delay(1000);
  //brake();
  direction = 5;
}

void goStraightMicro() {
  analogWrite(leftMotorSpeedControl, 255);
  analogWrite(rightMotorSpeedControl, 255);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(leftMotorForwardPin, HIGH);
  delay(20);
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
  delay(20);
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
  delay(20);
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

void checkFrontDistance() {
  frontDistance = sonar[1].ping_cm();
}

void checkLeftDistance() {
  leftDistance = sonar[0].ping_cm();
}

void checkRightDistance() {
  rightDistance = sonar[2].ping_cm();
}