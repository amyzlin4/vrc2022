/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Amy L                                                     */
/*    Created:      September 2022                                            */
/*    Description:  Full robot testing program: full driver control           */
/*                  and programming functions                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// MotorGroupR          motor_group   20, 19          
// MotorGroupL          motor_group   17, 18          
// Inertial             inertial      11              
// Controller1          controller                    
// flywheel             motor_group   1, 2            
// indexer              motor         4               
// intake               motor         3               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
// vex syntax - see forums for shortcuts
using namespace vex;

// A global instance of vex::competition ^
competition Competition;

// define your global instances of motors and other devices here
int flyC = 1;
int inC = 1;
bool toggleF = false;
bool lastF = false;
bool toggleI = false;
bool lastI = false;
bool revI = false;
int intVel = 100;
double DRIVE = 0.014;
double TURN = 8.91;

/* --------------------------------------------------------------------------*/
// DRIVER CONTROL FUNCTIONS:
/* --------------------------------------------------------------------------*/
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
int toggleFly() {  // use int for tasks (?)
  while(true) {
    if(Controller1.ButtonA.pressing() == true && !lastF) { 
    // if button is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
    } else if(Controller1.ButtonA.pressing() == false) {
    // else if button is not pressing:
      lastF = false;
    }
    if(toggleF) {
      flywheel.setVelocity(80,percent);
      flywheel.spin(forward);
    } else {
      flywheel.stop();
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
void ind() {
  indexer.setVelocity(100,percent);
  indexer.spinFor(forward,60,degrees);
  wait(400,msec);
  indexer.spinFor(reverse,60,degrees);
}
void intaker() {

  intake.setVelocity(100,percent);
  intake.spin(forward);

}
void stopIntaker() {
  intake.stop();
}
int toggleInt() {  // use int for tasks (?)
  while(true) {
    if(Controller1.ButtonB.pressing() == true && Controller1.ButtonX.pressing() == false && !lastI) { 
    // if buttonB is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = false; // set to forward
    } else if(Controller1.ButtonB.pressing() == false && Controller1.ButtonX.pressing() == true && !lastI) {
    // else if buttonX is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = true; // set to reverse
    } else if(Controller1.ButtonB.pressing() == false && Controller1.ButtonX.pressing() == false) {
      // else if buttons not pressing:
      lastI = false; // button was not pressed before
    }
    if(toggleI && revI == false) {
      // if toggle on and not reverse
      intVel = 100;
      intake.setVelocity(intVel,percent);
      intake.spin(forward);
    } else if(toggleI && revI == true) {
      // else if toggle on and reverse
      intVel = 80;
      intake.setVelocity(intVel,percent);
      intake.spin(reverse);
    } else {
      intake.stop();
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}

void revInt() {
  intake.setVelocity(80,percent);
  intake.spin(reverse);
}
void temp() {
  printf("Right Drive: %f\n", MotorGroupR.temperature(percent));
  printf("Left Drive: %f\n", MotorGroupL.temperature(percent));
  printf("Flywheel: %f\n", flywheel.temperature(percent));
  printf("Indexer: %f\n", indexer.temperature(percent));
  printf("Intake: %f\n", intake.temperature(percent));
}
void printVel() {
  printf("%f\n", flywheel.velocity(percent));
}
/* --------------------------------------------------------------------------*/
// PROG CONTROL FUNCTIONS:
/* --------------------------------------------------------------------------*/
void driveFor(double dis, int vel) {
  // update constants before testing!
  MotorGroupL.setVelocity(vel,percent);
  MotorGroupR.setVelocity(vel,percent);
  MotorGroupL.spinFor(forward,dis / DRIVE,degrees, false);
  MotorGroupR.spinFor(forward,dis / DRIVE,degrees, false);
}
void turnFor(double angl, int velT) {
  MotorGroupL.setVelocity(velT,percent);
  MotorGroupR.setVelocity(velT,percent);
  MotorGroupL.spinFor(forward,angl * TURN,degrees, false);
  MotorGroupR.spinFor(reverse,angl * TURN,degrees, false);
}
void driveTest() {
  driveFor(24, 20);
  wait(3, sec);
  turnFor(90, 40);
  wait(3, sec);
  driveFor(24, 20);
  wait(3, sec);
  turnFor(90, 40);
  wait(3, sec);
  driveFor(24, 20);
  wait(3, sec);
  turnFor(90, 40);
  wait(3, sec);
  driveFor(24, 20);
  wait(3, sec);
  turnFor(90, 40);
}
void driveTest2() {
  driveFor(-30, 30);
  wait(3, sec);
  turnFor(90, 40);
}
/* --------------------------------------------------------------------------*/
// Pre-Autonomous
/* --------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  Inertial.calibrate();
  flywheel.setStopping(coast);
  indexer.setStopping(hold);
  MotorGroupL.setStopping(coast);
  MotorGroupL.setStopping(coast);
}
/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/
void autonomous(void) {
   
}
/* --------------------------------------------------------------------------*/
// User Control
/* --------------------------------------------------------------------------*/
void usercontrol(void) {
  // create tasks outside of while loop
  task toggleFlyTask(toggleFly);
  task toggleIntTask(toggleInt);
  //task toggleRevTask(toggleRev);

  while (1) {
   

   wait(20, msec); 
  }
}

/* --------------------------------------------------------------------------*/
// Main will set up the competition functions and callbacks.
/* --------------------------------------------------------------------------*/
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
    //Controller1.ButtonA.pressed(fly);
    //Controller1.ButtonY.pressed(stopFly);
    //Controller1.ButtonX.pressed(intaker);
    //Controller1.ButtonB.pressed(stopIntaker);
    Controller1.ButtonDown.pressed(temp);
    Controller1.ButtonUp.pressed(driveTest);
    Controller1.ButtonLeft.pressed(driveTest2);
    Controller1.ButtonRight.pressed(printVel);
    
    
    wait(1, sec);
  }
}

/* --------------------------------------------------------------------------*/
// NOTES:
// remainder operation: must use int variables only - % sign 
// https://www.vexforum.com/t/programming-using-a-toggle-function/73119/8
//
//
/* --------------------------------------------------------------------------*/