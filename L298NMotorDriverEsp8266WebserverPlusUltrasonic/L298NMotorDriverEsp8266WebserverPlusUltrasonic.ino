/**
 * Code from : https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/
 */
/*
*  Source code for uploading to your ESP8266.
*  This program shows will provide a wifi access point and host a simple web page.  
*  When you connect to the accesspoint "ESP8266ROBOT" with password "ESP8266ROBOT", you can surf to the controllers page.
*  Your wifi card will get an IP and a gateway address (ip of the accesspoint).  Browse to http://192.168.4.1 (ip accesspoint)
*/



#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#ifndef APSSID
#define APSSID "ROBOMOW"
#define APPSK  "geheim123$"
#endif

/* Set these to your desired credentials. */
const char *ssid = "DINNO";
const char *password = "geheim123$";

String goDirection = "STOP";
String oldDirection = "STOP";
int oldSpeed = 0;
int goSpeed= 0;
String url;
String receivedData = "";

ESP8266WebServer server(80);

// Motor A connections
int enA = D1;
int in1 = D2;
int in2 = D3;
// Motor B connections
int in3 = D4;
int in4 = D5;
int enB = D6;

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
void handleRoot() {
  server.send(200, "text/html", "<h1>ESP8266 Robot</h1><br><a href=?direction=STOP>STOP</a><br><a href=?direction=FORWARD&speed=200>FORWARD</a><br><a href=?direction=BACKWARD&speed=200>BACKWARD</a><br><a href=?direction=LEFT&speed=200>LEFT</a><br><a href=?direction=ROTATELEFT&speed=200>ROTATE LEFT</a><br><a href=?direction=RIGHT&speed=200>RIGHT</a><br><a href=?direction=ROTATERIGHT&speed=200>ROTATE RIGHT</a>");
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.setTimeout(1000);
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(APSSID, APPSK);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  /*
   *  Wifi connect to network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  */
  
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");

    // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/////////////////////////////////////////////////////////////////////////

int speedVerify(int speed) {
  if (speed < 0){
    speed = 0;
  }
  if (speed > 255){
    speed = 255;
  }
  return speed;
}


void goForward(int motorSpeed){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  motorSpeed = speedVerify(motorSpeed);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  Serial.print("forward:");
  Serial.println(motorSpeed);
  
}

void goBackward(int motorSpeed){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  motorSpeed = speedVerify(motorSpeed);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  Serial.print("backward:");
  Serial.println(motorSpeed);
}
void goRight(int motorSpeed){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  motorSpeed = speedVerify(motorSpeed);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed/2);
  Serial.print("right:");
  Serial.println(motorSpeed);
  
}

void rotateRight(int motorSpeed){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  motorSpeed = speedVerify(motorSpeed);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  Serial.print("rotate right:");
  Serial.println(motorSpeed);
  
}


void goLeft(int motorSpeed){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
  analogWrite(enA, motorSpeed/2);
  analogWrite(enB, motorSpeed);
  Serial.print("left:");
  Serial.println(motorSpeed);
  
}
void rotateLeft(int motorSpeed){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  Serial.print("rotate left:");
  Serial.println(motorSpeed);
  
}

void Stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  Serial.println("stop");
}

void loop() {
  server.handleClient();
  if (server.hasArg("direction")) {
    goDirection = server.arg("direction");

    if (server.hasArg("speed")) {
      String speedArg  = server.arg("speed");
      goSpeed = speedArg.toInt();
      
    }
  }

  if (Serial.available() >=5) {
        receivedData = Serial.readStringUntil('$');
        if (receivedData.length() == 5) {
          Serial.println(receivedData);
          if (receivedData.substring(0,2) == "GF"){
            goDirection = "FORWARD";
          }
          if (receivedData.substring(0,2) == "GB"){
            goDirection = "BACKWARD";
          }
          if (receivedData.substring(0,2) == "GL"){
            goDirection = "LEFT";
          }
          if (receivedData.substring(0,2) == "GR"){
            goDirection = "RIGHT";
          }
          if (receivedData.substring(0,2) == "RL"){
            goDirection = "ROTATELEFT";
          }
          if (receivedData.substring(0,2) == "RR"){
            goDirection = "ROTATERIGHT";
          }
          if (receivedData.substring(0,2) == "SS"){
            goDirection = "STOP";
          }

          String speedData = receivedData.substring(2,5);
          goSpeed = speedData.toInt();

        }
        // empty sstring after $
        Serial.readString();
  }
    
  if (goDirection!=oldDirection or goSpeed!=oldSpeed) {
    if (goDirection == "FORWARD"){
      goForward(goSpeed);
    }
    if (goDirection == "BACKWARD"){
      goBackward(goSpeed);
    }
    if (goDirection == "LEFT"){
      goLeft(goSpeed);
    }
    if (goDirection == "ROTATELEFT"){
      rotateLeft(goSpeed);
    }

    if (goDirection == "RIGHT"){
      goRight(goSpeed);
    }
    if (goDirection == "ROTATERIGHT"){
      rotateRight(goSpeed);
    }

    if (goDirection == "STOP"){
      Stop();
    }
  }
  oldDirection=goDirection;
  oldSpeed = goSpeed;
  delay (15);
}
