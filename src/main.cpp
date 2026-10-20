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
bool isCascadeMoving = false;
int cascadeDirection = 1;

motor IntakeMotor = motor(PORT13, ratio18_1, false);
bool isIntakeMotorSpinning = false;
int intakeDirection = 1;

motor RotationalClaw = motor(PORT5, ratio18_1, false);
bool isRotationalClawSpinning = false;
int rotationalClawDirection = 1;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

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

void spinCascade(int direction) {
  if(isCascadeMoving && cascadeDirection == direction) {
    CascadeMotor.stop();
    isCascadeMoving = false;
  } else {
    cascadeDirection = direction;
    CascadeMotor.spin(forward, 11 * intakeDirection, volt);
    isCascadeMoving = true;
  }
}

void spinRotationalClaw(int direction) {
  if(isRotationalClawSpinning && rotationalClawDirection == direction) {
    RotationalClaw.stop();
    isRotationalClawSpinning = false;
  } else {
    rotationalClawDirection = direction;
    RotationalClaw.spin(forward, 11 * intakeDirection, volt);
    isRotationalClawSpinning = true;
  }
}

void spinIntakeForward() {
  spinIntake(1);
}

void spinIntakeBackward() {
  spinIntake(-1);
}

void raiseCascadeUp() {
  spinCascade(1);
}

void raiseCascadeDown() {
  spinCascade(-1);
}

void rotationalClawIntake() {
  spinRotationalClaw(1);
}

void rotationalClawOutake() {
  spinRotationalClaw(-1);
}

void setUp() {
  CascadeMotor.spin(forward, 1, volt);

  wait(2, sec);

  CascadeMotor.stop();

  CascadeMotor.setBrake(hold);

  CascadeMotor.setPosition(0, degrees);

  isSetUp = true;
}

void pre_auton(void) {
  setUp();
  Controller1.ButtonUp.pressed(spinIntakeForward);
  Controller1.ButtonDown.pressed(spinIntakeBackward);
  Controller1.ButtonR1.pressed(raiseCascadeUp);
  Controller1.ButtonR2.pressed(raiseCascadeDown);
  Controller1.ButtonL1.pressed(rotationalClawIntake);
  Controller1.ButtonL2.pressed(rotationalClawOutake);
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
  double rightMotorVolts = 10.0 * (Controller1.Axis3.position() - Controller1.Axis1.position()) / 90.0;
  double leftMotorVolts = 10.0 * (Controller1.Axis3.position() + Controller1.Axis1.position()) / 90.0;

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