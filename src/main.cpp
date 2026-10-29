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

motor CascadeMotor = motor(PORT15, ratio36_1, true);
bool isSetUp = false;
double LAYER_HEIGHT = 3;
double CASCADE_MIN_POSITION = 2.1;
double CASCADE_MAX_POSITION = 3 * LAYER_HEIGHT + CASCADE_MIN_POSITION;

double cascadeLayer = 0;

motor IntakeMotor = motor(PORT13, ratio18_1, false);

motor RotationalClaw = motor(PORT5, ratio18_1, true);

motor RotationalClawAngle = motor(PORT4, ratio36_1, true);
bool isRotationalClawScoring = false;

digital_out Toggle = digital_out(Brain.ThreeWirePort.A);

inertial RotationalClawSensor = inertial(PORT7);
inertial InertialSensor = inertial(PORT3);


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

// These are the helper functions for managing the intake

void spinCascadeToPos(double turn, bool waitForCompletion = false) {
  if(turn < 0) {
    CascadeMotor.spinTo(0, turns, waitForCompletion);
  } else if(turn > CASCADE_MAX_POSITION) {
    CascadeMotor.spinTo(CASCADE_MAX_POSITION, turns, waitForCompletion);
  } else {
    CascadeMotor.spinTo(turn, turns, waitForCompletion);
  }
}

void raiseCascade() {
  cascadeLayer++;
  spinCascadeToPos(cascadeLayer * LAYER_HEIGHT, true);
}

void lowerCascade() {
  cascadeLayer--;
  spinCascadeToPos(cascadeLayer * LAYER_HEIGHT, true);
}

void intakeRotationalClaw() {
  RotationalClaw.spin(forward, 11, volt);
}

void outakeRotationalClaw() {
  RotationalClaw.spin(forward, -11, volt);
}

void stopRotationalClaw() {
  RotationalClaw.stop();
}

double getRotationalClawAngle() {
  return -RotationalClawSensor.orientation(roll, deg);
}

void rotationalClawAnglePositionToLoading(bool waitForCompletion = false) {
  double angle = getRotationalClawAngle();
  while(angle > 2) {
    RotationalClawAngle.spin(reverse);
    wait(5, msec);
    angle = getRotationalClawAngle();
  }
  RotationalClawAngle.stop();
}

void rotationalClawAnglePositionToScoring(bool waitForCompletion = false) {
  double angle = getRotationalClawAngle();
  while(angle < 90) {
    RotationalClawAngle.spin(forward);
    wait(5, msec);
    angle = getRotationalClawAngle();
  }
  while(angle > 95) {
    RotationalClawAngle.spin(reverse);
    wait(5, msec);
    angle = getRotationalClawAngle();
  }
  RotationalClawAngle.stop();
}

void rotationalClawAnglePositionToMaxScoring(bool waitForCompletion = false) {
  double angle = getRotationalClawAngle();
  while(angle < 140) {
    RotationalClawAngle.spin(forward);
    wait(5, msec);
    angle = getRotationalClawAngle();
  }
  while(angle > 145) {
    RotationalClawAngle.spin(reverse);
    wait(5, msec);
    angle = getRotationalClawAngle();
  }
  RotationalClawAngle.stop();
}

void spinIntakeForward() {
  IntakeMotor.spin(forward, 11, volt);
}

void spinIntakeReverse() {
  IntakeMotor.spin(reverse, 11, volt);
}

//These are the state functions

void setToRestState() {
  intakeState = 0;
  cascadeLayer = 0;
  RotationalClaw.stop();
  IntakeMotor.stop();
  CascadeMotor.spinToPosition(0, degrees, true);

  rotationalClawAnglePositionToLoading(true);
}

void setToIntakeState() {
  intakeState = 1;
  setToRestState();
  intakeRotationalClaw();
  spinIntakeForward();
}

void setToOutakeState() {
  intakeState = -1;
  setToRestState();
  outakeRotationalClaw();
  spinIntakeReverse();
}

void setToRaiseState() {
  intakeState = 2;
  setToRestState();
  rotationalClawAnglePositionToScoring();
  raiseCascade();
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

void setToMaxScoreState() {
  intakeState = 4;
  outakeRotationalClaw();
  rotationalClawAnglePositionToMaxScoring();
}

// These functions are called by the controller

void flipToggle() {
  Toggle.set(!Toggle.value());
}

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
  if(intakeState == -3) {
    stopRotationalClaw();
  }

  if(!isCascadeUp()) {
    setToRaiseState();
  } else {
    rotationalClawAnglePositionToScoring();
    raiseCascade();
  }
  intakeState = 2;
}

void lowerClaw() {
  if(!isCascadeUp()) {
    return;
  }
  if(intakeState == 3) {
    stopRotationalClaw();
  }
  intakeState = -2;
  if(cascadeLayer == 0) {
    setToRestState();
  } else {
    rotationalClawAnglePositionToScoring();
    lowerCascade();
  }
}

void score() {
  if(!isCascadeUp()) { return; }

  if(cascadeLayer < 3) {
    setToScoreState();
  } else {
    setToMaxScoreState();
  }
}

void unscore() {
  if(isCascadeUp()) {
    setToUnscoreState();
  }
}

// These functions sets up the robot

void setUp() {
  isSetUp = true;

  RotationalClawAngle.setBrake(hold);
  RotationalClawAngle.setPosition(0, degrees);

  CascadeMotor.setBrake(hold);
  CascadeMotor.setVelocity(100, percent);

  FrontLeftDrive.setVelocity(100, percent);
  FrontRightDrive.setVelocity(100, percent);
  BackLeftDrive.setVelocity(100, percent);
  BackRightDrive.setVelocity(100, percent);

  FrontLeftDrive.setBrake(hold);
  FrontRightDrive.setBrake(hold);
  BackLeftDrive.setBrake(hold);
  BackRightDrive.setBrake(hold);

  CascadeMotor.spin(forward, 2, volt);

  wait(2, sec);

  CascadeMotor.stop();

  CascadeMotor.setPosition(0, degrees);
}

void pre_auton(void) {
  thread one = thread(setUp);
  Controller1.ButtonL1.pressed(intake);
  Controller1.ButtonL2.pressed(outake);
  Controller1.ButtonR1.pressed(raiseClaw);
  Controller1.ButtonR2.pressed(lowerClaw);
  Controller1.ButtonDown.pressed(unscore);
  Controller1.ButtonUp.pressed(score);
  Controller1.ButtonX.pressed(flipToggle);
  Controller1.ButtonB.pressed(setToRestState);

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

void driveForward(double feet) {
  double feetInTurns = feet * 24.0 / 15.0;
  FrontLeftDrive.spinFor(feetInTurns, turns, false);
  BackLeftDrive.spinFor(feetInTurns, turns, false);
  FrontRightDrive.spinFor(feetInTurns, turns, false);
  BackRightDrive.spinFor(feetInTurns, turns);
}

void turnRight(double degree) {
  double newAngle = degree + InertialSensor.rotation(deg);

  while(InertialSensor.rotation(deg) < newAngle) {
    FrontLeftDrive.spin(reverse);
    BackLeftDrive.spin(reverse);
    FrontRightDrive.spin(forward);
    BackRightDrive.spin(forward);

    wait(5, msec);
  }

  FrontLeftDrive.stop();
  BackLeftDrive.stop();
  FrontRightDrive.stop();
  BackRightDrive.stop();
}

int autonomousState = 0;

void autonomousScoringThread() {
  wait(2, seconds);
  rotationalClawAnglePositionToScoring();

  while (autonomousState < 1) {
    wait(20, msec);
  }

  outakeRotationalClaw();
  wait(1, sec);
  
  autonomousState = 2;
  
}

void autonomous(void) {
  thread aThread = thread(autonomousScoringThread);

  flipToggle();
  wait(0.5, seconds);
  flipToggle();
  driveForward(-0.5);
  turnRight(90);
  driveForward(-19.0 / 12.0);
  autonomousState = 1;

  while (autonomousState < 2) {
    wait(20, msec);
  }

  driveForward(0.5);
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
  double rightMotorVolts = 10.0 * (Controller1.Axis3.position() + Controller1.Axis4.position()) / 90.0;
  double leftMotorVolts = 10.0 * (Controller1.Axis3.position() - Controller1.Axis4.position()) / 90.0;

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