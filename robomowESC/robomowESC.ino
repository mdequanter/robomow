/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>
Servo motor1;  // create servo object to control a servo
Servo motor2;  // create servo object to control a servo
Servo motor3;  // create servo object to control a servo

int rcThrottlePin = 3;
int rcSpeedPin = 5;
int rcTurnPin = 6;
int motor1Pin = 9;
int motor2Pin = 10;
int motor3Pin = 11;

int rcMax = 1880;
int rcMin = 1110;

int escMax = 2200;
int escMin = 800;

int lastMotorspeed = 0;

bool forward = true; 

int trottle = 0;
int turn = 0;
int motorspeed = 0;
int value;

void setup() {
  Serial.begin(9600);
  motor1.attach(motor1Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor1.writeMicroseconds(1500);
  motor2.attach(motor2Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor2.writeMicroseconds(1500);
  motor3.attach(motor3Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor3.writeMicroseconds(1500);

  delay(1000);
}




void loop() {
  trottle = (pulseIn (rcThrottlePin, HIGH, 100000));
  turn = (pulseIn (rcTurnPin, HIGH, 100000));
  motorspeed = (pulseIn (rcSpeedPin, HIGH, 100000));
/*
  motor1.writeMicroseconds(1600);
  motor2.writeMicroseconds(1600);
  delay(20);

  if (motorspeed > 1600) {
    motor1.writeMicroseconds(1800);
    motor2.writeMicroseconds(1800);    
  }
*/
  
  
  //Serial.print("trottle:");
  //Serial.println(trottle);
  //Serial.print("turn:");
  //Serial.println(turn);
  //Serial.print("motorspeed:");
  //Serial.println(motorspeed);

  if (trottle > 1000 ) {
    value =  map(trottle, rcMin, rcMax, escMin, escMax);
    motor3.writeMicroseconds(value);
    delay(20);
  }     

  

  if (motorspeed > 1550) {
    value =  map(motorspeed, rcMin, rcMax, escMin, escMax);    
    int motor1Value = value;
    int motor2Value = value;
    if (turn > 1600) {
      motor1Value = 1500;
    }
    if (turn < 1400) {
      motor2Value = 1500;
    }
    
    motor1.writeMicroseconds(motor1Value);
    motor2.writeMicroseconds(motor2Value);
    lastMotorspeed = motorspeed;
    delay(20);
    //Serial.println("forward:");   
    //Serial.println(value);
    forward = true;
       
  }

  if (motorspeed < 1470) {
    if (forward == true) {
      motor1.writeMicroseconds(1500);
      motor2.writeMicroseconds(1500);
      delay(200);
      motor1.writeMicroseconds(800);
      motor2.writeMicroseconds(800);
      delay(200);
      motor1.writeMicroseconds(1500);
      motor2.writeMicroseconds(1500);
      delay(200);
      forward = false; 
    }
    value =  map(motorspeed, rcMin, rcMax, escMin, escMax);
    int motor1Value = value;
    int motor2Value = value;
    if (turn > 1600) {
      motor1Value = 1500;
    }
    if (turn < 1400) {
      motor2Value = 1500;
    }
    
    motor1.writeMicroseconds(motor1Value);
    motor2.writeMicroseconds(motor2Value);
    lastMotorspeed = motorspeed;
    delay(20);  
    //Serial.println("backward"); 
    //Serial.println(value);
    forward = false;
  }

  if (motorspeed > 1470 and motorspeed < 1550 ){
      motor1.writeMicroseconds(1500);
      motor2.writeMicroseconds(1500);
  }
  
  
  /*
  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500);
  delay(2000);
  motor1.writeMicroseconds(2200);
  motor2.writeMicroseconds(2200);
  delay(2000);
  motor1.writeMicroseconds(1800);
  motor2.writeMicroseconds(1800);  
  delay(2000);
  motor1.writeMicroseconds(1600);
  motor2.writeMicroseconds(1600);
  delay(2000);
  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500);
  delay(2000);
  motor1.writeMicroseconds(800);
  motor2.writeMicroseconds(800);
  delay(500);
  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500);
  delay(2000);
  motor1.writeMicroseconds(1200);
  motor2.writeMicroseconds(1200);
  delay(2000);
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  delay(2000);
  motor1.writeMicroseconds(800);
  motor2.writeMicroseconds(800);
  delay(2000);
  */
}
