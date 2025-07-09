#include <Servo.h>

Servo leftMotor, rightMotor;

void setup() {
  Serial.begin(115200); // For ESP32, 115200 is typical
  leftMotor.attach(9);   // PWM output to RC Input 1
  rightMotor.attach(10); // PWM output to RC Input 2
  Serial.println("ESP32 Ready");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "Forward") {
      leftMotor.writeMicroseconds(2000);
      rightMotor.writeMicroseconds(2000);
      Serial.println("Moving forward");
    }
    else if (command == "Backward") {
      leftMotor.writeMicroseconds(1000);
      rightMotor.writeMicroseconds(1000);
      Serial.println("Moving backward");
    }
    else if (command == "Neutral") {
      leftMotor.writeMicroseconds(1500);
      rightMotor.writeMicroseconds(1500);
      Serial.println("Stopped");
    }
    else {
      Serial.println("Unknown command");
    }
  }
}