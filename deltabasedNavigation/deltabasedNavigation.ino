#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <Servo.h>

uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false;                                   // set this to true to use the remote ID

boolean use_processing = false;                         // set this to true to output data for the processing sketch

uint8_t num_anchors = 4;                                    // the number of anchors

uint16_t anchors[4] = {0x6a38,0x6a15, 0x6a21, 0x6a40};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {0, 4750, 4750, 0};               // anchor x-coorindates in mm
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
int headingMarge = 2;
int destX,destY,destZ;
int destH = 180;
int rightTurn = 0;
int leftTurn = 0;
int lastX;
int lastY;
double lastDestDistance = 0;

// tag variables

int tagX,tagY,tagZ = 0;
int tagH = 0;

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
  if(!remote){
    remote_id = NULL;
  }
  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setAnchorsManual();
  delay(2000);

  motor1.attach(motor1Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor1.writeMicroseconds(1500);
  motor2.attach(motor2Pin,escMin,escMax);  // attaches the servo on pin 9 to the servo object
  motor2.writeMicroseconds(1500);
  delay(1000);

  
  last_millis = millis();
  delay(10);  
}

void loop(){
  coordinates_t position;
  int status;
  sensor_raw_t sensor_raw;
  uint8_t calibration_status = 0;
  int dt;
  status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  if (status == POZYX_SUCCESS){
    getCoordinates(position);
  }

  if (Serial.available()) {
    commando = Serial.readString(); 
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
  //Serial.print("X: ");
  double destDistance = calculateDistance(tagX,tagY,destX,destY);
  double headingToDestination = calculateHeading(tagX,tagY,destX,destY);
  headingToDestination = 180 - abs(headingToDestination);
  double headingToLastPosition = calculateHeading(tagX,tagY,lastX,lastY);
  
  if (abs(destDistance - lastDestDistance) > 10 ) {
  Serial.print(tagX);
  Serial.print(",Y: ");
  Serial.print(tagY);
  Serial.print(",DestinationDistance: ");
  Serial.print(",");
  Serial.print(destDistance);
  Serial.print(",HeadingtoDestination: ");
  Serial.print(headingToDestination);
  Serial.print(",HeadingtoLast: ");
  Serial.println(headingToLastPosition);
  }

  lastX = tagX;
  lastY = tagY;
  lastDestDistance = destDistance;

  delay(100);

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
  return head;
}

double calculateDistance(double x1, double y1, double x2,double y2) {
  long distance1;
  x1 = x1/100;
  x2 = x2/100;
  y1 = y1/100;
  y2 = y2/100;
  distance1 = (y2-y1)*(y2-y1)+(x2-x1)*(x2-x1);
  double distance;
  distance =  sqrt(distance1);
  return distance*100;
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
