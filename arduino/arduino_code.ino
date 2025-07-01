#include <Servo.h>

Servo leftMotor, rightMotor;

const int PWM_MIN = 1100;
const int PWM_MAX = 1900;

void setup() {
  Serial.begin(9600);
  leftMotor.attach(9);  // Pin 9 connected to RC Input 1
  rightMotor.attach(10); // Pin 10 connected to RC Input 2
  while(!Serial);
  Serial.println("Arduino Ready");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("Forward")){
      Serial.println("Starting Forward Command...");
      leftMotor.writeMicroseconds(2000);
      rightMotor.writeMicroseconds(2000);
      delay(1000);
      Serial.println("Forward Command Ended...");
    }else if (command.startsWith("Backward")){
      Serial.println("Starting Backward Command...");
      leftMotor.writeMicroseconds(1000);
      rightMotor.writeMicroseconds(1000);
      delay(1000);
      Serial.println("Backward Command Ended...");
    }else if (command.startsWith("Neutral")){
      Serial.println("Starting Neutral Command...");
      leftMotor.writeMicroseconds(1500);
      rightMotor.writeMicroseconds(1500);
      delay(1000);
      Serial.println("Neutral Command Ended...");
    }else{
      Serial.println("Unknown Command");
    }
  }
}
