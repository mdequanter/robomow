// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  The Pozyx ready to localize tutorial (c) Pozyx Labs

  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino

  This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
  of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
  parameters and upload this sketch. Watch the coordinates change as you move your device around!
*/
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

bool demoMode = false;

uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false;                                   // set this to true to use the remote ID

boolean use_processing = false;                         // set this to true to output data for the processing sketch

uint8_t num_anchors = 4;                                    // the number of anchors

uint16_t anchors[4] = {0x6a38,0x6a15, 0x6a21, 0x6a40};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {0, 4500, 4500, 0};               // anchor x-coorindates in mm
int32_t anchors_y[4] = {0, 0, 2850, 2850};                  // anchor y-coordinates in mm
int32_t heights[4] = {920, 920, 920, 920};              // anchor z-coordinates in mm

/*
// buiten
uint16_t anchors[4] = {0x6a21, 0x6a40, 0x6a38, 0x6a15};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {0, 10250, 10080, 0};               // anchor x-coorindates in mm
int32_t anchors_y[4] = {0, 0, 12750, 12750};                  // anchor y-coordinates in mm
int32_t heights[4] = {1200, 1200, 1200, 1200};              // anchor z-coordinates in mm
*/



uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 1000;                                  // height of device, required in 2.5D positioning

String commando = ""; 


// tag variables

int tagX,tagY,tagZ = 0;
int tagH = 0;

// destinations

int destX,destY,destZ,destH;
double destHeading,destDistance;
double arrivalMarge = 10;

//int headingTable[360];
int pointTo = 0;


////////  Motors ///////////////////

Servo motor1;  // create servo object to control a servo
Servo motor2;  // create servo object to control a servo
Servo motor3;  // create servo object to control a servo

int escMax = 2200;
int escMin = 800;

int motor1Pin = 9;
int motor2Pin = 10;
int motor3Pin = 11;



void setup(){
  Serial.begin(115200);


  motor1.attach(motor1Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor1.writeMicroseconds(1500);
  motor2.attach(motor2Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor2.writeMicroseconds(1500);
  motor3.attach(motor3Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor3.writeMicroseconds(1500);

  delay(1000);


  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  if(!remote){
    remote_id = NULL;
  }
  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setAnchorsManual();
  printCalibrationResult();
  delay(2000);

  tagX,tagY,tagH = 0;
  
  Serial.println(F("Starting positioning: "));
}

void loop(){
  coordinates_t position;
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
      String part = commando.substring(posX+1, posY);
      destX = part.toInt();
      part = commando.substring(posY+1,posH);
      destY = part.toInt();    
      part = commando.substring(posH+1);
      destH = part.toInt();
  }
  
  
  if(remote){
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
  }else{
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  }

  if (status == POZYX_SUCCESS or demoMode == true){
    // prints out the result
    if (demoMode == false) {
      getCoordinates(position);
    }
    //printTagCoordinates();
    destHeading = calculateHeading(tagX,tagY,destX,destY);
    destDistance = calculateDistance(tagX,tagY,destX,destY);
    
    if (destDistance < arrivalMarge ) {
      //Serial.println("-----------arrived to destination-----------");
      //Serial.print("set heading: ");
      //Serial.println(destH);
    } else {
      //Serial.println("----------------heading to destination-----------------");
      // set motors


     if (destHeading <= 137 ) {
      pointTo = 150-destHeading;
     }
     
     if (destHeading >= 138 ) {
      pointTo = 360-destHeading;
     }
     /*
     Serial.print("X: ");
     Serial.print(tagX);
     Serial.print(",Y: ");
     Serial.print(tagY);
     Serial.print(",H: ");
     Serial.print(tagH);
     Serial.print("distance:");
     Serial.print(destDistance);
     Serial.print(", heading:");
     Serial.print(destHeading);
     Serial.print(", Point to:");
     Serial.println(pointTo);
     */
      Serial.print("X: ");
      Serial.print(tagX);
      Serial.print(",Y: ");
      Serial.print(tagY);


     int headingDiff = abs(pointTo-tagH); 
     if (headingDiff > 20 ) {
      if (pointTo <= tagH) {
        Serial.println("links");
        motor1.writeMicroseconds(1800);
        motor2.writeMicroseconds(1500);
      }
      if (pointTo > tagH) {
        Serial.println("rechts");
        motor1.writeMicroseconds(1500);
        motor2.writeMicroseconds(1800);
      }
      
      Serial.print("turning : ");
      Serial.println(headingDiff);

     } else {
      Serial.print("distance:");
      Serial.println(destDistance);
      if (destDistance > 5) {
        Serial.println("rechtdoor");
        motor2.writeMicroseconds(1800);
        motor1.writeMicroseconds(1800);
      } else {
        Serial.println("stop");
        motor2.writeMicroseconds(1500);
        motor1.writeMicroseconds(1500);
      }
     }

     


     int roundedHeading = round(tagH);
     int roundDestination = round(destHeading);
     //headingTable[roundDestination] =  roundedHeading ;
     
    }  
  }else{
    // prints out the error code
    //printErrorCode("positioning");
  }
}


double calculateDistance(double x1, double y1, double x2,double y2) {
  long distance1;

  if (x1>0) {
    x1 = x1/100;
  }
  if (x2>0) {
    x2 = x2/100;
  }
  if (y1>0) {
    y1 = y1/100;
  }
  if (y2>0) {
    y2 = y2/100;
  }  
  distance1 = (y2-y1)*(y2-y1)+(x2-x1)*(x2-x1);
  double distance;
  distance =  sqrt(distance1);
  return distance*100;
}


double calculateHeading(int x1,int y1,int x2,int y2) {
  double head;
  head = atan2((y2-y1), (x2-x1))*(180/3.141592);
  
  return head+180;
}



// prints the coordinates for either humans or for processing
void getCoordinates(coordinates_t coor){
  euler_angles_t orientation;
  sensor_raw_t sensor_raw;
  Pozyx.getEulerAngles_deg(&orientation, remote_id);
  tagH = orientation.heading;

  //tagH = sensor_raw.magnetic[1];
  
  
  uint16_t network_id = remote_id;

  if (network_id == NULL){
    network_id = 0;
  }
  tagX = coor.x;
  tagY = coor.y;
  tagZ = coor.z;
}


void printTagCoordinates(){
    Serial.print("x: ");
    Serial.print(tagX);
    Serial.print(" y: ");
    Serial.print(tagY);
    Serial.print(" z: ");
    Serial.print(tagZ);
    Serial.print(" h: ");
    Serial.println(tagH);    
}

// error printing function for debugging
void printErrorCode(String operation){
  uint8_t error_code;
  if (remote_id == NULL){
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if(status == POZYX_SUCCESS){
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(remote_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);
  }else{
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);
  Serial.print("list size: ");
  Serial.println(status*list_size);

  if(list_size == 0){
    printErrorCode("configuration");
    return;
  }

  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);

  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);

  coordinates_t anchor_coor;
  
  for(int i = 0; i < list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.print(anchor_coor.z);
  }
}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}
