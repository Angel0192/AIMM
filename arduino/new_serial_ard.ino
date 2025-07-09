#include <ESP32Servo.h>

Servo leftMotor, rightMotor;

ESP32PWM pwm;

String inputString = "";

int leftMotor = 10;
int rightMotor = 9;

void setup() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  Serial.begin(115200);  // Make sure this matches your Python script
  pwm.attachPin(leftMotor, 1000,10);  
  pwm.attachPin(rightMotor, 1000,10);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      // Parse the string: format is "x,z"
      float linear = 0.0;
      float angular = 0.0;

      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        linear = inputString.substring(0, commaIndex).toFloat();
        angular = inputString.substring(commaIndex + 1).toFloat();
      }

      // Simple mixing to get left/right speeds
      int left_speed = constrain((linear - angular) * 90 + 90, 0, 180);
      int right_speed = constrain((linear + angular) * 90 + 90, 0, 180);

      leftMotor.writeScaled(left_speed);
      rightMotor.write(right_speed);

      inputString = "";  // Clear buffer
    } else {
      inputString += c;
    }
  }
}
