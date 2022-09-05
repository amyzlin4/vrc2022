/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// MotorGroupR          motor_group   4, 1            
// MotorGroupL          motor_group   3, 2            
// Inertial             inertial      17              
// Controller1          controller                    
// flywheel             motor_group   8, 9            
// indexer              motor         10              
// intake               motor         11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
int flyC = 1;
int inC = 1;



// DRIVER CONTROL FUNCTIONS:
void driveCtrl() {
  MotorGroupL.setVelocity(Controller1.Axis3.position(percent), percent);
  MotorGroupR.setVelocity(Controller1.Axis3.position(percent), percent);
  MotorGroupL.spin(forward);
  MotorGroupR.spin(forward);
}
void driveTurn() {
  MotorGroupL.setVelocity(Controller1.Axis1.position(percent), percent);
  MotorGroupR.setVelocity(Controller1.Axis1.position(percent), percent);
  MotorGroupL.spin(forward);
  MotorGroupR.spin(reverse);
}
void fly() {
  if(flyC % 2 == 0) {
    flywheel.stop();
  } 
  else {
    flywheel.setVelocity(100,percent);
    flywheel.spin(forward);
  }
  flyC ++;
}
void ind() {
  indexer.setVelocity(100,percent);
  indexer.spinFor(forward,60,degrees);
  wait(400,msec);
  indexer.spinFor(reverse,60,degrees);
}
void intaker() {
  if((inC % 2) == 0) {
    intake.stop();
  } 
  else {
    intake.setVelocity(100,percent);
    intake.spin(forward);
  }
  inC ++;
}
void temp() {
  printf("Right Drive: %f\n", MotorGroupR.temperature(percent));
  printf("Left Drive: %f\n", MotorGroupL.temperature(percent));
  printf("Flywheel: %f\n", flywheel.temperature(percent));
  printf("Indexer: %f\n", indexer.temperature(percent));
  printf("Intake: %f\n", intake.temperature(percent));
}
// -------------------------------------------------------------------
// PROG CONTROL FUNCTIONS:
void driveFor(double dis, int vel) {
  MotorGroupL.setVelocity(vel,percent);
  MotorGroupR.setVelocity(vel,percent);
  MotorGroupL.spinFor(forward,dis,degrees);
  MotorGroupR.spinFor(forward,dis,degrees);
}
void turnFor(double angl, int velT) {
  MotorGroupL.setVelocity(velT,percent);
  MotorGroupR.setVelocity(velT,percent);
  MotorGroupL.spinFor(forward,angl,degrees);
  MotorGroupR.spinFor(reverse,angl,degrees);
}
// -------------------------------------------------------------------

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  Inertial.calibrate();
  flywheel.setStopping(coast);
  indexer.setStopping(hold);
}



void autonomous(void) {
  
}



void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
   

   wait(20, msec); 
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
    Controller1.Axis3.changed(driveCtrl);
    Controller1.Axis1.changed(driveTurn);
    Controller1.ButtonR1.pressed(ind);
    Controller1.ButtonA.pressed(fly);
    Controller1.ButtonB.pressed(intaker);
    Controller1.ButtonY.pressed(temp);
    wait(100, msec);
  }
}

/* NOTES:
// remainder operation: must use int variables only - % sign 
// 
*/