/*
  Basic Robot Arm Controller
  robot-arm-control-basic.ino
  Controls 5 DOF Robot Arm, uses Arduino Nano and PCA9685 PWM Controller
  Uses Adafruit PWM library
  Uses 5 potentiometers for input (can be modified for 6)

  DroneBot Workshop 2018
  https://dronebotworkshop.com
*/

// Include Wire Library for I2C Communications
#include <Wire.h>

// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define Potentiometer Inputs

int controlBase = A0;
int controlElbow = A1;
int controlWrist = A2;
int controlPivot = A3;
int controlJaws = A6;

// Define Motor Outputs on PCA9685 board

int motorBase = 0;
int motorElbow = 1;
int motorWrist = 2;
int motorPivot = 3;
int motorJaws = 4;

// Define Motor position variables
int mtrDegreeBase;
int mtrDegreeElbow;
int mtrDegreeWrist;
int mtrDegreePivot;
int mtrDegreeJaws;

void setup()
{
  // Setup PWM Controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}

// Function to move motor to specific position
void moveMotorDeg(int moveDegree, int motorOut)
{
  int pulse_wide, pulse_width;

  // Convert to pulse width
  pulse_wide = map(moveDegree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}

// Function to convert potentiometer position into servo angle
int getDegree(int controlIn)
{
  int potVal, srvDegree;

  // Read values from potentiometer
  potVal = analogRead(controlIn);

  // Calculate angle in degrees
  srvDegree = map(potVal, 0, 1023, 0, 180);

  // Return angle in degrees
  return srvDegree;

}

void loop() {

  //Control Base Motor

  // Get desired position
  mtrDegreeBase = getDegree(controlBase);
  // Move motor
  moveMotorDeg(mtrDegreeBase, motorBase);


  //Control Elbow Motor

  // Get desired position
  mtrDegreeElbow = getDegree(controlElbow);
  // Move motor
  moveMotorDeg(mtrDegreeElbow, motorElbow);


  //Control Wrist Motor

  // Get desired position
  mtrDegreeWrist = getDegree(controlWrist);
  // Move motor
  moveMotorDeg(mtrDegreeWrist, motorWrist);


  //Control Pivot Motor

  // Get desired position
  mtrDegreePivot = getDegree(controlPivot);
  // Move motor
  moveMotorDeg(mtrDegreePivot, motorPivot);


  //Control Jaws Motor

  // Get desired position
  mtrDegreeJaws = getDegree(controlJaws);
  // Move motor
  moveMotorDeg(mtrDegreeJaws, motorJaws);


  // Add short delay
  delay(20);

}
