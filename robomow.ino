/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01

    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#define enA 5
#define enB 6

#define in1 12
#define in2 13


void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  analogWrite(enA, 255); // Send PWM signal to motor A
  analogWrite(enB, 255); // Send PWM signal to motor B
  delay(100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  delay(1000);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(100);
  analogWrite(enA, 255); // Send PWM signal to motor A
  analogWrite(enB, 255); // Send PWM signal to motor B

  delay(1000);
  
  
}
