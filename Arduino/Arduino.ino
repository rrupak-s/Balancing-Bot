#include "pid.h"          // Include PID library for PID controller
#include "motor.h"        // Include motor library for motor control
#include <MPU6050.h>      // Include MPU6050 library for accelerometer and gyroscope

#define PIN_SWITCH A0     // Pin for switch input
#define M1_DIR1 A3        // Motor 1 direction pin 1
#define M1_DIR2 A2        // Motor 1 direction pin 2
#define M1_PWM 6          // Motor 1 PWM pin

#define M2_DIR2 7         // Motor 2 direction pin 2
#define M2_DIR1 4         // Motor 2 direction pin 1
#define M2_PWM 5          // Motor 2 PWM pin

const float kp = 7;        // Proportional gain for PID controller
const float ki = 2.75;     // Integral gain for PID controller
const float kd = 0.0;      // Derivative gain for PID controller
const float setpoint = 0.0; // Target setpoint for pitch angle

PID controller(kp, ki, kd);  // Create PID controller object
Motor motor_L(M1_DIR1, M1_DIR2, M1_PWM);  // Create motor 1 object
Motor motor_R(M2_DIR1, M2_DIR2, M2_PWM);  // Create motor 2 object
MPU6050 mpu;  // Create MPU6050 object for accelerometer and gyroscope

volatile uint16_t counter = 0;  // Counter variable for timing
const uint8_t dt = 10;  // Time interval in milliseconds
const float max_pitch = 37;  // Maximum allowable pitch angle
const float min_pitch = -37; // Minimum allowable pitch angle

void setup() {
  pinMode(PIN_SWITCH, INPUT_PULLUP);  // Set switch pin as input with internal pull-up resistor

  controller.setOutputLimits(255, 0);  // Set PID output limits (max, min)
  Serial.begin(115200);  // Start serial communication at 115200 baud
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {  // Initialize MPU6050 sensor
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Wait until switch is pressed
  while (digitalRead(PIN_SWITCH)) {}  // Wait for the switch to be pressed
  counter = millis();  // Initialize counter with current time
}

void loop() {
  if (millis() - counter > dt) {  // Execute loop at specified interval (dt)
    counter = millis();  // Update counter with current time

    // Read sensor data
    Vector normAccel = mpu.readNormalizeAccel();  // Read normalized accelerometer data

    // Calculate Pitch angle
    float pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
    Serial.print(" Pitch = ");
    Serial.print(pitch);

    // Update PID controller
    float error = setpoint - pitch;  // Calculate error (difference from setpoint)
    float output;

    if (pitch > max_pitch || pitch < min_pitch) {  // Check if pitch is outside allowable range
      output = 0;  // If outside range, set output to 0 (stop motors)
    } else {
      output = controller.update(error, (float)dt / 1000.0);  // Update PID controller with error and time interval
    }
    Serial.print(" Output = ");
    Serial.print(output);
    Serial.println();

    // Actuate motors
    motor_L.drive(output);  // Apply PID output to control motor 1
    motor_R.drive(output);  // Apply PID output to control motor 2
  }
}
