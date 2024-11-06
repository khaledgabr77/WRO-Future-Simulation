#include <Arduino.h>

const int servoPin = 9;  // Pin for servo control
const int motorPin = 10; // Pin for motor driver control

// Limit variables
const float steerMin = -3.14;  // Minimum steering angle in radians
const float steerMax = 3.14;   // Maximum steering angle in radians
const float driveMin = 0;       // Minimum motor speed in rad/s
const float driveMax = 10;      // Maximum motor speed in rad/s

void setup() {
    Serial.begin(9600);
    pinMode(servoPin, OUTPUT);
    pinMode(motorPin, OUTPUT);
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil(':');
        String valueStr = Serial.readStringUntil('\n');

        float value = valueStr.toFloat();
        
        if (command == "S") {
            // Limit steering angle
            value = constrain(value, steerMin, steerMax);
            int servoPWM = map(value, steerMin, steerMax, 0, 180); // Assuming steering angle range is -π to π
            analogWrite(servoPin, servoPWM);
        } else if (command == "D") {
            // Limit speed
            value = constrain(value, driveMin, driveMax);
            int motorPWM = map(value, driveMin, driveMax, 0, 255); // Assuming speed range is 0 to 10 rad/s
            analogWrite(motorPin, motorPWM);
        }
    }
}