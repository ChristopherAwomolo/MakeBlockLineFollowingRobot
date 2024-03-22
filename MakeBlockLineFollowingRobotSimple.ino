#include "MeMegaPi.h"

MeUltrasonicSensor ultraSensor(PORT_7); 
int thresholdDistance = 10; // Threshold distance

MeLineFollower lineFinder(PORT6);
MeMegaPiDCMotor motor1(PORT1A); // Left motor
MeMegaPiDCMotor motor2(PORT1B); // Left motor
MeMegaPiDCMotor motor3(PORT2A); // Right motor
MeMegaPiDCMotor motor4(PORT2B); // Right motor

uint8_t motorSpeed = 50;
uint8_t diffFactor = 50;
uint8_t diffFactorBackwards = 25; // Adjust this based on the difference between motors

void setup() {
  Serial.begin(9600);
}

void loop() {
  int distance = ultraSensor.distanceCm();

  // Check if the distance is below the threshold distance
  if (distance <= thresholdDistance) {
    Serial.println("Object too close! Stopping the car.");
    // Stop the car
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  } else {
    // Continue with line following if distance is not below the threshold
    int sensorState = lineFinder.readSensors();
    switch(sensorState) {
      case S1_IN_S2_IN: // Both sensors are inside the line
        Serial.println("Both sensors are inside the line");
        // Move forward at adjusted speed
        motor1.run(motorSpeed);              // Left motor
        motor2.run(motorSpeed);              // Left motor
        motor3.run(motorSpeed + diffFactor); // Right motor
        motor4.run(motorSpeed + diffFactor); // Right motor
        break;
      case S1_IN_S2_OUT: // Left sensor is inside, right sensor is outside
        Serial.println("Sensor 1 is inside, Sensor 2 is outside");
        // Adjust to the left
        motor1.run(-motorSpeed);              // Left motor
        motor2.run(-motorSpeed);              // Left motor
        motor3.run(motorSpeed+ diffFactor);              // Right motor
        motor4.run(motorSpeed+ diffFactor);              // Right motor
        break;
      case S1_OUT_S2_IN: // Right sensor is inside, left sensor is outside
        Serial.println("Sensor 1 is outside, Sensor 2 is inside");
        // Adjust to the right
        motor1.run(motorSpeed+ diffFactor); // Left motor
        motor2.run(motorSpeed+ diffFactor); // Left motor
        motor3.run(-motorSpeed);              // Right motor
        motor4.run(-motorSpeed);              // Right motor
        break;
      case S1_OUT_S2_OUT: // Both sensors are outside the line
        Serial.println("Both sensors are outside the line");
        // Stop
        motor1.run(-motorSpeed+diffFactorBackwards);              // Left motor
        motor2.run(-motorSpeed+diffFactorBackwards);              // Left motor
        motor3.run(-motorSpeed); // Right motor
        motor4.run(-motorSpeed); // Right motor
        break;
      default: 
        motor1.run(-motorSpeed+diffFactorBackwards);              // Left motor
        motor2.run(-motorSpeed+diffFactorBackwards);              // Left motor
        motor3.run(-motorSpeed); // Right motor
        motor4.run(-motorSpeed); // Right motor
        break;
    }
  }

  delay(200); // Adjust delay as necessary for responsiveness
}
