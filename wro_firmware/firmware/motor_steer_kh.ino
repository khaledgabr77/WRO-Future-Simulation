#include <Servo.h>

// Motor control pins
#define L298N_enA 9   // PWM pin for motor speed
#define L298N_in1 12  // Motor direction pin 1
#define L298N_in2 13  // Motor direction pin 2

// Servo control pin
#define servoPin 6    // PWM pin for servo control

// Variables to store velocity and angle commands
double velocity = 0.0;  // in rad/s
double angle = 0.0;     // in radians

// Variables for serial communication
bool is_velocity_cmd = false;
bool is_angle_cmd = false;
char value[20];  // Buffer to store incoming command values
uint8_t value_idx = 0;

Servo myServo;

void setup() {
  // Initialize motor control pins
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  // Initialize servo
  myServo.attach(servoPin);
  myServo.write(90);  // Center the servo

  Serial.begin(115200);  // Start serial communication
}

void loop() {
  // Read and interpret commands from serial input
  while (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'v') {  // Velocity command
      is_velocity_cmd = true;
      is_angle_cmd = false;
      value_idx = 0;
      memset(value, 0, sizeof(value));
    }
    else if (chr == 'a') {  // Angle command
      is_velocity_cmd = false;
      is_angle_cmd = true;
      value_idx = 0;
      memset(value, 0, sizeof(value));
    }
    else if (chr == ',') {  // Command separator
      if (is_velocity_cmd) {
        velocity = atof(value);  // Convert velocity value to double
      }
      else if (is_angle_cmd) {
        angle = atof(value);  // Convert angle value to double
        // Convert angle in radians to servo angle in degrees
        // Assuming servo moves from 0 to 180 degrees
        double angle_deg = angle * (180.0 / 3.14159265358979323846); // Convert radians to degrees
        int servo_angle = map(angle_deg, -45, 45, 0, 180);  // Map angle to servo range
        servo_angle = constrain(servo_angle, 0, 180);  // Ensure servo angle is within valid range
        myServo.write(servo_angle);  // Set servo position
      }
      // Reset command flags and buffer for next command
      is_velocity_cmd = false;
      is_angle_cmd = false;
      value_idx = 0;
      memset(value, 0, sizeof(value));
    }
    else {
      // Accumulate the value characters
      if (value_idx < sizeof(value) - 1) {
        value[value_idx++] = chr;
        value[value_idx] = '\0';  // Null-terminate the string
      }
    }
  }

  // Control motor speed and direction based on velocity value
  if (velocity >= 0) {
    // Forward direction
    digitalWrite(L298N_in1, HIGH);
    digitalWrite(L298N_in2, LOW);
  }
  else {
    // Reverse direction
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, HIGH);
  }

  // Convert velocity in rad/s to PWM value
  // Define maximum velocity (rad/s) corresponding to PWM value 255
  double max_velocity = 10.0;  // Adjust this value based on your motor's specs

  // Calculate PWM value
  int pwm_value = (int)(abs(velocity) * 255.0 / max_velocity);
  pwm_value = constrain(pwm_value, 0, 255);  // Ensure PWM value is within 0-255
  analogWrite(L298N_enA, pwm_value);

  delay(10);  // Small delay for stability (adjust as needed)
}
