#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

// IMU sensor
Adafruit_MPU6050 mpu;

// PID variables
double setpoint = 0;  // Desired angle (upright position)
double input;         // Current angle from IMU
double output;        // PID output

// PID tuning parameters
double Kp = 2.0;
double Ki = 5.0;
double Kd = 1.0;

// PID object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor control pins
const int steeringActuatorPin = 9;
const int rearWheelActuatorPin = 10;

// Encoder pin
const int encoderPin = 2;

// Encoder variables
volatile long encoderCount = 0;
double wheelSpeed = 0;
double desiredSpeed = 0; // Set desired speed as needed

void setup() {
  Serial.begin(115200);

  // Initialize IMU sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  // Initialize motor control pins
  pinMode(steeringActuatorPin, OUTPUT);
  pinMode(rearWheelActuatorPin, OUTPUT);

  // Initialize encoder pin
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
}

void loop() {
  // Read IMU sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate tilt angle (simplified)
  input = atan2(a.acceleration_y, a.acceleration_z) * 180 / PI;

  // Compute PID
  myPID.Compute();

  // Apply PID output to the steering actuator
  if (output > 0) {
    analogWrite(steeringActuatorPin, output);
  } else {
    analogWrite(steeringActuatorPin, -output);
  }

  // Control the rear wheel speed to match desired speed
  // Simple proportional control for the rear wheel
  double speedError = desiredSpeed - wheelSpeed;
  int rearWheelOutput = constrain(speedError * Kp, -255, 255);
  
  if (rearWheelOutput > 0) {
    analogWrite(rearWheelActuatorPin, rearWheelOutput);
  } else {
    analogWrite(rearWheelActuatorPin, -rearWheelOutput);
  }

  // Debugging output
  Serial.print("Angle: ");
  Serial.print(input);
  Serial.print(" PID Output: ");
  Serial.print(output);
  Serial.print(" Wheel Speed: ");
  Serial.print(wheelSpeed);
  Serial.print(" Rear Wheel Output: ");
  Serial.println(rearWheelOutput);

  delay(10);
}

// Encoder ISR
void encoderISR() {
  encoderCount++;
}

// Calculate wheel speed
void calculateWheelSpeed() {
  static long lastCount = 0;
  static unsigned long lastTime = 0;

  unsigned long currentTime = millis();
  long countDifference = encoderCount - lastCount;
  double timeDifference = (currentTime - lastTime) / 1000.0;

  wheelSpeed = (countDifference / timeDifference) * (2 * PI * wheelRadius) / encoderTicksPerRevolution;

  lastCount = encoderCount;
  lastTime = currentTime;
}

