/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       elisur                                                    */
/*    Created:      9/15/2026, 3:16:58 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

brain Brain;
controller Controller1 = controller(primary);

motor BackRightDrive = motor(PORT11, ratio18_1, true);
motor BackLeftDrive = motor(PORT1, ratio18_1, false);
motor FrontRightDrive = motor(PORT12, ratio18_1, false);
motor FrontLeftDrive = motor(PORT2, ratio18_1, true);

motor CascadeMotor = motor(PORT15, ratio18_1, true);
bool isSetUp = false;

motor IntakeMotor = motor(PORT13, ratio18_1, false);
bool isIntakeMotorSpinning = false;
int intakeDirection = 1;

motor RotationalClaw = motor(PORT5, ratio18_1, true);
bool isRotationalClawSpinning = false;
int rotationalClawDirection = 1;

motor RotationalClawAngle = motor(PORT4, ratio36_1, true);
bool isRotationalClawScoring = false;


/* 0: in rest position
 * 1: intaking
 * -1: outaking
 * 2: raising
 * -2: lowering
 * 3: score
 * -3: unscore
*/
int intakeState = 0;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

// These functions draw the texas flag

bool aboveLine(double x, double y, double px1, double py1, double px2, double py2) {
  return (py2 - py1) * (x - px1) / (px2 - px1) + py1 <= y;
}

void drawStar() {
  double pi = 3.1415926535;
  double xVals[5] = {cos(pi / 10.0), 0, cos(pi - pi / 10.0), cos(pi + 3 * pi / 10), cos(2 * pi - 3 * pi / 10)};
  double yVals[5] = {sin(pi / 10.0), 1, sin(pi - pi / 10.0), sin(pi + 3 * pi / 10), sin(2 * pi - 3 * pi / 10)};

  for(int x = 0; x < 239; x++) {
    double mappedX = (x * 2.0 / 239.0 - 1) * -1;
    for(int y = 10; y < 249; y++) {
      double mappedY = ((y-10) * 2.0 / 239.0 - 1) * -1;

      if(mappedY > yVals[0]) {
        if(!aboveLine(mappedX, mappedY, xVals[1], yVals[1], xVals[3], yVals[3]) && !aboveLine(mappedX, mappedY, xVals[1], yVals[1], xVals[4], yVals[4])) {
          Brain.Screen.drawPixel(x, y);
        }
      } else if(aboveLine(mappedX, mappedY, xVals[2], yVals[2], xVals[4], yVals[4])) {
        if(aboveLine(mappedX, mappedY, xVals[0], yVals[0], xVals[3], yVals[3])) {
          Brain.Screen.drawPixel(x, y);
        } else if(!aboveLine(mappedX, mappedY, xVals[1], yVals[1], xVals[4], yVals[4])) {
          Brain.Screen.drawPixel(x, y);
        }
      } else if(aboveLine(mappedX, mappedY, xVals[0], yVals[0], xVals[3], yVals[3]) && !aboveLine(mappedX, mappedY, xVals[1], yVals[1], xVals[3], yVals[3])) {
        Brain.Screen.drawPixel(x, y);
      }
    }
  }
}

void drawTexanFlag() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(0, 0, 239, 239);
  Brain.Screen.setFillColor(white);
  Brain.Screen.drawRectangle(239, 0, 239, 239 / 2);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(239, 239 / 2, 239, 239);
  
  Brain.Screen.setFillColor(white);
  drawStar();
}

void spinIntake(int direction) {
  if(isIntakeMotorSpinning && intakeDirection == direction) {
    IntakeMotor.stop();
    isIntakeMotorSpinning = false;
  } else {
    intakeDirection = direction;
    IntakeMotor.spin(forward, 11 * intakeDirection, volt);
    isIntakeMotorSpinning = true;
  }
}

// These are the helper functions for managing the intake

void stopIntake() {
  IntakeMotor.stop();
  isIntakeMotorSpinning = false;
}

void raiseCascade() {
  CascadeMotor.spinTo(CascadeMotor.position(turns) + 6, turns);
}

void lowerCascade() {
  double newPosition = CascadeMotor.position(turns) - 6;
  if(newPosition < 0) {
    newPosition = 0;
  }
  CascadeMotor.spinTo(newPosition, turns);
}

void cascadeToScoringPosition(bool waitForCompletion = false) {
  CascadeMotor.spinTo(0, turns, waitForCompletion);
}

void intakeRotationalClaw() {
  isRotationalClawSpinning = true;
  rotationalClawDirection = 1;
  RotationalClaw.spin(forward, 11 * rotationalClawDirection, volt);
}

void outakeRotationalClaw() {
  isRotationalClawSpinning = true;
  rotationalClawDirection = -1;
  RotationalClaw.spin(forward, 11 * rotationalClawDirection, volt);
}

void stopRotationalClaw() {
  isRotationalClawSpinning = false;
  RotationalClaw.stop();
}

void rotationalClawAnglePositionToLoading() {
  RotationalClawAngle.spinToPosition(0, degrees, false);
}

void rotationalClawAnglePositionToScoring() {
  RotationalClawAngle.spinToPosition(190, degrees, false);
}

//These are the state functions

void setToRestState() {
  intakeState = 0;
  stopRotationalClaw();
  stopIntake();
  cascadeToScoringPosition(true);
  rotationalClawAnglePositionToLoading();
}

void setToIntakeState() {
  intakeState = 1;
  setToRestState();
  intakeRotationalClaw();
  spinIntake(1);
}

void setToOutakeState() {
  intakeState = -1;
  setToRestState();
  outakeRotationalClaw();
  spinIntake(-1);
}

void setToRaiseState() {
  intakeState = 2;
  setToRestState();
  rotationalClawAnglePositionToScoring();
  CascadeMotor.spinTo(2, turns, true);
}

void setToScoreState() {
  intakeState = 3;
  outakeRotationalClaw();
  raiseCascade();
}

void setToUnscoreState() {
  intakeState = -3;
  intakeRotationalClaw();
  lowerCascade();
}

// These functions are called by the controller

void intake() {
  if(intakeState != 1) {
    setToIntakeState();
  } else {
    setToRestState();
  }
}

void outake() {
  if(intakeState != -1) {
    setToOutakeState();
  } else {
    setToRestState();
  }
}

bool isCascadeUp() {
  return intakeState == 2 || intakeState == -2 || intakeState == 3 || intakeState == -3;
}

void raiseClaw() {
  if(!isCascadeUp()) {
    setToRaiseState();
  } else {
    raiseCascade();
  }
  intakeState = 2;
}

void lowerClaw() {
  if(!isCascadeUp()) {
    return;
  }
  intakeState = -2;
  if(CascadeMotor.position(turns) <= 6) {
    setToRestState();
  } else {
    lowerCascade();
  }
}

void score() {
  if(isCascadeUp()) {
    setToScoreState();
  }
}

void unscore() {
  if(isCascadeUp()) {
    setToUnscoreState();
  }
}

// These functions sets up the robot

void setUp() {
  CascadeMotor.spin(forward, 1, volt);

  wait(2, sec);

  CascadeMotor.stop();

  RotationalClawAngle.setBrake(hold);
  RotationalClawAngle.setPosition(0, degrees);

  CascadeMotor.setBrake(hold);
  CascadeMotor.setVelocity(100, percent);

  CascadeMotor.setPosition(0, degrees);

  isSetUp = true;
}

void pre_auton(void) {
  setUp();
  Controller1.ButtonL1.pressed(intake);
  Controller1.ButtonL2.pressed(outake);
  Controller1.ButtonR1.pressed(raiseClaw);
  Controller1.ButtonR2.pressed(lowerClaw);
  Controller1.ButtonDown.pressed(unscore);
  Controller1.ButtonUp.pressed(score);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

void moveForward(){
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void updateDriveMotors() {
  double rightMotorVolts = 10.0 * (Controller1.Axis2.position() + Controller1.Axis1.position()) / 90.0;
  double leftMotorVolts = 10.0 * (Controller1.Axis2.position() - Controller1.Axis1.position()) / 90.0;

  BackRightDrive.spin(forward, rightMotorVolts, volt);
  FrontRightDrive.spin(forward, rightMotorVolts, volt);

  BackLeftDrive.spin(forward, leftMotorVolts, volt);
  FrontLeftDrive.spin(forward, leftMotorVolts, volt);
}

void usercontrol(void) {
  // User control code here, inside the loop
  if(!isSetUp) {
    setUp();
  }
  drawTexanFlag();
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    updateDriveMotors();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}