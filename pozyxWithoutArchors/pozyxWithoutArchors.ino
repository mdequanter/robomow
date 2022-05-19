/**
  The pozyx ranging demo (c) Pozyx Labs
  please check out https://www.pozyx.io/Documentation/Tutorials/getting_started/Arduino
  
  This demo requires one (or two) pozyx shields and one Arduino. It demonstrates the 3D orientation and the functionality
  to remotely read register data from a pozyx device. Place one of the pozyx shields on the Arduino and upload this sketch. 
  
  This demo reads the following sensor data: 
  - pressure
  - acceleration
  - magnetic field strength
  - angular velocity
  - the heading, roll and pitch
  - the quaternion rotation describing the 3D orientation of the device. This can be used to transform from the body coordinate system to the world coordinate system.
  - the linear acceleration (the acceleration excluding gravity)
  - the gravitational vector
  
  The data can be viewed in the Processing sketch orientation_3D.pde 
*/

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <Servo.h>


String commando = ""; 
int headingMarge = 2;
int destX,destY,destZ;
int destH = 180;
int rightTurn = 0;
int leftTurn = 0;
int X = 0;
int Y = 0;



///// servo parameters /////// 
Servo motor1;  // create servo object to control a servo
Servo motor2;  // create servo object to control a servo
int escMax = 2200;
int escMin = 800;
int motor1Pin = 9;
int motor2Pin = 10;






////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint32_t last_millis;                 // used to compute the measurement interval in milliseconds 

////////////////////////////////////////////////

void setup()
{  
  Serial.begin(115200);
    
  if(Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_IMU) == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }  

  motor1.attach(motor1Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor1.writeMicroseconds(1500);
  motor2.attach(motor2Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor2.writeMicroseconds(1500);
  delay(1000);

  
  last_millis = millis();
  delay(10);  
}

void loop(){
  sensor_raw_t sensor_raw;
  uint8_t calibration_status = 0;
  int dt;
  int status;


  if (Serial.available()) {
    commando = Serial.readString(); 
      /*
      for (int i = 0; i<=359;i++) {
        Serial.print(i);
        Serial.print(";");
        Serial.println(headingTable[i]);
      }
      */

      int posX = commando.indexOf('X', 0);
      int posY = commando.indexOf('Y', 1);
      int posH = commando.indexOf('H', 1);
      int posCurrentX = commando.indexOf('A', 1);
      int posCurrentY = commando.indexOf('B', 1);   
      String part = commando.substring(posX+1, posY);
      destX = part.toInt();
      part = commando.substring(posY+1,posH);
      destY = part.toInt();    
      part = commando.substring(posH+1);
      destH = part.toInt();
      part = commando.substring(posCurrentX+1,posCurrentY);
      X = part.toInt();
      part = commando.substring(posCurrentY+1);
      Y = part.toInt();      
  }





  
  
  if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 10) == POZYX_SUCCESS){
    Pozyx.getRawSensorData(&sensor_raw);
    Pozyx.getCalibrationStatus(&calibration_status);
  }else{
    uint8_t interrupt_status = 0;
    Pozyx.getInterruptStatus(&interrupt_status);
    return;
  }
  

  dt = millis() - last_millis;
  last_millis += dt;    
  // print time difference between last measurement in ms, sensor data, and calibration data
  //Serial.print(dt, DEC);
  //Serial.print(",");
  int TagDirection = getDirection(sensor_raw);
  Serial.print(X);
  Serial.print(",");
  Serial.print(Y);
  Serial.print(",");
  Serial.print(TagDirection);
  double DirectionToGoal = calculateHeading(X,Y,destX,destY);
  Serial.print(",");
  Serial.println(DirectionToGoal);
  destH = DirectionToGoal;
}

/*
 * Get direction to go
 * 0 forward
 * 1 right
 * -1 left
 * 
 */

int getDirection(sensor_raw_t sensor_raw){
  int trueDegrees = map(sensor_raw.euler_angles[0], 0, 5750, 0, 360);
  int difference = destH-trueDegrees;
  Serial.print(trueDegrees);
  Serial.print("->");
  Serial.println(destH);

  if (abs(trueDegrees-destH) > headingMarge) {
    if (trueDegrees < destH) {
      rightTurn = destH-trueDegrees;
      leftTurn = trueDegrees+(360-destH); 
    }
    if (trueDegrees > destH) {
      leftTurn = trueDegrees-destH;
      rightTurn = destH+(360-trueDegrees); 
    }
  
    if (leftTurn < rightTurn) {
      return -1;  
    }
  
    if (leftTurn >= rightTurn) {
      return 1;   
    }    
  } else {
    return 0;
  }
}

double calculateHeading(int x1,int y1,int x2,int y2) {
  double head;
  head = atan2((y2-y1), (x2-x1))*(180/3.141592);

  head = head+120;

  if (head > 360) {
    head = head-360;
  }

  if (head < 0) {
    head = 360-abs(head);
  }
  return head;
}


void printCalibrationStatus(uint8_t calibration_status){
  Serial.print(calibration_status & 0x03);
  Serial.print(",");
  Serial.print((calibration_status & 0x0C) >> 2);
  Serial.print(",");
  Serial.print((calibration_status & 0x30) >> 4);
  Serial.print(",");
  Serial.print((calibration_status & 0xC0) >> 6);  
}
