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
// #include <String.h>

sensor_raw_t sensor_raw;


////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

boolean remote = false;               // boolean to indicate if we want to read sensor data from the attached pozyx shield (value 0) or from a remote pozyx device (value 1)
uint16_t remote_id = 0x6606;          // the network id of the other pozyx device: fill in the network id of the other device
uint32_t last_millis;                 // used to compute the measurement interval in milliseconds 

char *degrees;

int wantedHeading = 999;
int standardDirection = 999;



///////////  Motor Shield  /////////////////////

int ENA = 5; //M1 speed control
int IN1 = 6; //M1 Direction1
int IN2 = 7; //M1 Direction2
int ENB = 9; //M1 Direction Control
int IN3 = 10; //M1 Direction Control
int IN4 = 11; //M2 Direction Control

int relaisPin = 8; //Pin naar relay voor grasmaaier bladen

int basicSpeed = 250;






////////////////////////////////////////////////


void turnAround(int turnDirection = 1) {

  if (turnDirection == 1) {
    stopMotors();
    delay(1000);
    gobackward();
    delay(1000);
    stopMotors();
    delay(1000);
    turnright();
    delay(1500);
    stopMotors();
    delay(1000);
    goforward();
    delay(1000);
    stopMotors();
    delay(1000);
    turnright();
    delay(1000);
    stopMotors();
    delay(1000);
    goforward();
    delay(1000);    
  }
  if (turnDirection == 2) {
    stopMotors();
    delay(1000);
    gobackward();
    delay(1000);
    stopMotors();
    delay(1000);
    turnleft();
    delay(1500);
    stopMotors();
    delay(1000);
    goforward();
    delay(1000);
    stopMotors();
    delay(1000);
    turnleft();
    delay(1000);
    stopMotors();
    delay(1000);
    goforward();
    delay(1000);    
  }  

}




void setup()
{  
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

    
  if(Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_IMU) == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }

  if(!remote)
    remote_id = NULL;
  
  last_millis = millis();
  delay(10);  
}

void loop(){
  uint8_t calibration_status = 0;
  int dt;
  int status;
  if(remote){
     status = Pozyx.getRawSensorData(&sensor_raw, remote_id);
     status &= Pozyx.getCalibrationStatus(&calibration_status, remote_id);
    if(status != POZYX_SUCCESS){
      return;
    }
  }else{
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 10) == POZYX_SUCCESS){
      Pozyx.getRawSensorData(&sensor_raw);
      Pozyx.getCalibrationStatus(&calibration_status);
    }else{
      uint8_t interrupt_status = 0;
      Pozyx.getInterruptStatus(&interrupt_status);
      return;
    }
  }

  dt = millis() - last_millis;
  last_millis += dt;    
  // print time difference between last measurement in ms, sensor data, and calibration data
  //Serial.print(dt, DEC);
  //Serial.print(",");
  
  int degrees = getRawSensorData(sensor_raw); 
  

  // If first time then go straight
  if (wantedHeading == 999) {
   wantedHeading =  degrees;
   standardDirection = degrees;
  }



  while (Serial.available() > 0) {
    wantedHeading = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.print("New heading:");
      Serial.println(wantedHeading);
    }
  }
  int distance = readPerimeterFront();
  Serial.print("Perimeter front:");
  Serial.println(distance);

  
  if (distance > 200) {
    if (wantedHeading == standardDirection) {
      turnAround(1);
      if (wantedHeading > 180) {
        wantedHeading = standardDirection - 180;
      }
      if (wantedHeading <= 180) {
        wantedHeading = standardDirection + 180;
      }
    } else {
      turnAround(2);
      wantedHeading = standardDirection;
    }
  }
  Serial.println(degrees);
  followHeading(degrees, wantedHeading);




}

int getRawSensorData(sensor_raw_t sensor_raw){
  int value = map(sensor_raw.euler_angles[0], 0, 5759, 0, 359);
  //Serial.print(value);
  return value;
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

/////////////  Driving routines  ///////////////////




void gotoHeading(int heading, int wantedHeading) {
  
  int ccw = 0;
  int cw = 0;
  int leftSpeed = basicSpeed;
  int rightSpeed = basicSpeed-6;
  
  //Serial.print(heading);
  //Serial.print("->");
  //Serial.print(wantedHeading);


  if (wantedHeading > heading ) {
    cw = wantedHeading - heading;
    ccw = heading + (360 - wantedHeading);
  }

  if (wantedHeading < heading ) {
    cw = (360 - heading) + wantedHeading;
    ccw = wantedHeading - heading;
  }
  
  if (abs(cw) < abs(ccw)) {
    int difference = abs(cw);
    if (difference > 5) {
        turnright();
    }
  }

  if (abs(ccw) < abs(cw)) {
    int difference = abs(cw);
    if (difference > 5) {
        turnleft();
      }
  }    

}




void followHeading(int heading , int wantedHeading) {

  int ccw = 0;
  int cw = 0;
  int leftSpeed = basicSpeed;
  int rightSpeed = basicSpeed-6;
  
  //Serial.print(heading);
  //Serial.print("->");
  //Serial.print(wantedHeading);


  if (wantedHeading > heading ) {
    cw = wantedHeading - heading;
    ccw = heading + (360 - wantedHeading);
  }

  if (wantedHeading < heading ) {
    cw = (360 - heading) + wantedHeading;
    ccw = wantedHeading - heading;
  }
  
  //Serial.print(",cw difference: ");
  //Serial.print(cw);
  //Serial.print(",ccw difference: ");
  //Serial.println(ccw);


  //goforward();


  if (abs(cw) < abs(ccw)) {
    int difference = abs(cw);
    if (difference > 10) {
      if (difference > 20) {
        turnright();
      } else {
        stearRight(difference);  
      } 
    } else {
      goforward();
    }
  }

  if (abs(ccw) < abs(cw)) {
    int difference = abs(cw);
    if (difference > 10) {
      if (difference > 20) {
        turnleft();
      } else {
        stearLeft(difference);  
      }     
    } else {
      goforward();
    }
  }

  
}



///////////// Go Forward Routine ///////////////////
void goforward(){
  Serial.println("Forward");
  analogWrite (ENA,basicSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite (ENB,basicSpeed-5);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);

  }
////////////////////////////////////////////////////


///////////// Go Backward Routine ///////////////////
void gobackward(){
  Serial.println("Backward");
  analogWrite (ENA,basicSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);  
  analogWrite (ENB,basicSpeed);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  }
////////////////////////////////////////////////////


///////////// Turn Right Routine ///////////////////  
  void turnright(){
  Serial.println("Right");
  analogWrite (ENA,basicSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite (ENB,35);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  }
////////////////////////////////////////////////////


///////////// StearRight /////////////////// 
  void stearRight(int difference){
  
    Serial.println("stearRight");
    if (difference > 20) {
      difference=20;
    }
    //difference = map(difference,0,20,0,20);
    int leftSpeed = leftSpeed + difference;  
    
    analogWrite (ENA,leftSpeed);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite (ENB,basicSpeed);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  
  }

////////////////////////////////////////////////////


///////////// StearLeft /////////////////// 
  void stearLeft(int difference){
    
    Serial.println("stearLeft");
    if (difference > 20) {
      difference=20;
    }
    //difference = map(difference,0,20,0,20);
    int rightSpeed = rightSpeed + difference;  
    
    analogWrite (ENA,basicSpeed);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite (ENB,rightSpeed);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    
  }

////////////////////////////////////////////////////




///////////// Turn Left Routine /////////////////// 
  void turnleft(){
  Serial.println("Left");
  analogWrite (ENA,basicSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite (ENB,basicSpeed);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  }
////////////////////////////////////////////////////


///////////// Stop Routine ///////////////////  
  void stopMotors(){
  Serial.println("Stop");
  analogWrite (ENA,0);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  analogWrite (ENB,0);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  }
////////////////////////////////////////////////////


///////////// Perimeter wire detection ///////////////////  
/* 
  Get data from perimeter pins and put them in periDataFront and periDataRight
  https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/
  https://forum.arduino.cc/t/analogread-always-returning-1023/239329/10
  De rightpin geeft 0 als niets in de buurt en 6 vannaf rond de 7 cm
*/

int readPerimeterFront(){

  int periDataFront = analogRead(A0);
  Serial.print("SensorDataFront: ");
  Serial.println(periDataFront);
  return periDataFront;
}
