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
// driveR               motor_group   12, 11          
// driveL               motor_group   18, 13          
// flywheel             motor_group   4, 6            
// intake               motor         3               
// indexer              motor         7               
// Controller1          controller                    
// roller               digital_out   H               
// expand               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

int flyC = 1;
int inC = 1;
bool toggleF = false;
bool lastF = false;
bool slowF = false;
bool toggleI = false;
bool lastI = false;
bool revI = false;
int intVel = 100;
double DRIVE = 0.014;
double TURN = 8.91;
bool lastP = false;
bool toggleP = false;
bool enableDriver = true;
bool enableInt = true;
bool toggleR = false;
bool lastR = false;

// DRIVER CONTROL FUNCTIONS:
void driver() {
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent), percent);
  driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent), percent);
  driveL.spin(forward);
  driveR.spin(forward);
}
int toggleFly() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonY.pressing() == true && Controller1.ButtonA.pressing() == false && !lastF) { 
    // if buttonA is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = false; // set to normal speed
    } else if(Controller1.ButtonY.pressing() == false && Controller1.ButtonA.pressing() == true && !lastF) {
    // else if buttonY is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = true; // set to slower speed
    } else if(Controller1.ButtonA.pressing() == false && Controller1.ButtonY.pressing() == false) {
    // else if button is not pressing:
      lastF = false; // button was not pressed before
      enableInt = true;
    }
    if(toggleF && slowF == false) {
      // if toggle on and not slow
      // flywheel.setVelocity(85,percent);
      // flywheel.spin(forward);
      flywheel.spin(forward,8.0,voltageUnits::volt);
      enableInt = false;
    } else if(toggleF && slowF == true) {
      // if toggle on and slow
      // flywheel.setVelocity(65,percent);
      // flywheel.spin(forward);
      flywheel.spin(forward,6.6,voltageUnits::volt);
      enableInt = false;
    } else {
      flywheel.stop();
      enableInt = true;
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
void ind() {
  indexer.setVelocity(100,percent);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.setPosition(0,degrees);
  indexer.spinFor(reverse,80,degrees);
  
}
void intaker() {

  intake.setVelocity(100,percent);
  intake.spin(forward);

}
void stopIntaker() {
  intake.stop();
}
int toggleRoll() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonL1.pressing() == true) { 
    // if buttonA is pressing:
      roller.set(true);
    } else {
    // else if button is not pressing:
      roller.set(false);
    }
  
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
int toggleInt() {  // use int for tasks (?)
  while(enableDriver) {
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
    if(toggleI && revI == false && enableInt) {
      // if toggle on and not reverse
      intVel = 100;
      intake.setVelocity(intVel,percent);
      intake.spin(forward);
      flywheel.spin(reverse,1,voltageUnits::volt);
    } else if(toggleI && revI == true && enableInt) {
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

void expansion() {
  if(enableDriver) {
    expand.set(true);
    wait(1,sec);
    expand.set(false);
  }
}

void temp() {
  printf("Right Drive: %f\n", driveR.temperature(percent));
  printf("Left Drive: %f\n", driveL.temperature(percent));
  printf("Flywheel: %f\n", flywheel.temperature(percent));
  printf("Indexer: %f\n", indexer.temperature(percent));
  printf("Intake: %f\n", intake.temperature(percent));
  
  if(indexer.temperature(percent) >= 60) {
    Brain.Screen.print("indexer: ");
    Brain.Screen.print(indexer.temperature(percent));
    Brain.Screen.newLine();
  }
  if(intake.temperature(percent) >= 60) {
    Brain.Screen.print("intake: ");
    Brain.Screen.print(intake.temperature(percent));
    Brain.Screen.newLine();
  }
  if(flywheel.temperature(percent) >= 60) {
    Brain.Screen.print("flywheel: ");
    Brain.Screen.print(flywheel.temperature(percent));
    Brain.Screen.newLine();
  }
  if(driveR.temperature(percent) >= 60) {
    Brain.Screen.print("right drive: ");
    Brain.Screen.print(driveR.temperature(percent));
    Brain.Screen.newLine();
  }
  if(driveL.temperature(percent) >= 60) {
    Brain.Screen.print("left drive: ");
    Brain.Screen.print(driveL.temperature(percent));
    Brain.Screen.newLine();
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  enableDriver = false;
  // Inertial.calibrate();
  flywheel.setStopping(coast);
  indexer.setStopping(hold);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
}

/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/

void auto3v1() {
  // 2 tile side
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  driveL.setVelocity(55,percent);
  driveR.setVelocity(55,percent);
  driveR.spinFor(reverse,2450,degrees, false);
  driveL.spinFor(reverse,1000,degrees, false);
  wait(2, sec);
  flywheel.spin(forward,7.66,voltageUnits::volt);
  roller.set(true);
  wait(0.3, sec);
  driveL.setVelocity(40,percent);
  driveR.setVelocity(40,percent);
  driveL.spinFor(forward,110,degrees,false);
  driveR.spinFor(forward,110,degrees,false);
  wait(0.8, sec);
  roller.set(false);
  wait(0.3, sec);
  driveL.spinFor(forward,200,degrees);
  wait(0.9, sec);
 
  wait(1.5, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.spin(forward,7.9,voltageUnits::volt);
  wait(2.5, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.stop();
  
}

void auto3v2() {
  // 2 tile side
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  driveL.setVelocity(55,percent);
  driveR.setVelocity(55,percent);
  driveR.spinFor(reverse,2450,degrees, false);
  driveL.spinFor(reverse,1000,degrees, false);
  wait(2, sec);
  roller.set(true);
  wait(0.3, sec);
  driveL.setVelocity(40,percent);
  driveR.setVelocity(40,percent);
  driveL.spinFor(forward,110,degrees,false);
  driveR.spinFor(forward,110,degrees,false);
  wait(0.8, sec);
  roller.set(false);
  wait(0.3, sec);
  driveL.spinFor(forward,170,degrees);
  wait(0.9, sec);
  flywheel.spin(forward,8.1,voltageUnits::volt);
  // flywheel.setVelocity(64,percent);
  // flywheel.spin(forward);
  wait(1.5, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.spin(forward,7.8,voltageUnits::volt);
  wait(0.9, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.stop();
  // turn to intake
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveL.spinFor(reverse,1.2,turns,false);
  driveR.spinFor(forward,1.2,turns,false);
  wait(1.2, sec);
  // intake 1
  intake.setVelocity(100,percent);
  intake.spin(forward);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  flywheel.spin(reverse,1,voltageUnits::volt);
  driveL.spinFor(forward,9,turns,false);
  driveR.spinFor(forward,9,turns,false);
  wait(4, sec);

  // turn to shoot
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  intake.stop();
  flywheel.spin(forward,7.5,voltageUnits::volt);
  driveL.spinFor(forward,1.9,turns,false);
  driveR.spinFor(reverse,1.9,turns,false);
  wait(2, sec);
  // shoot 1

  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);

  // flywheel.spin(forward,6.84,voltageUnits::volt);
  // wait(0.9, sec);
  // indexer.spinFor(forward,80,degrees);
  // wait(100,msec);
  // indexer.spinFor(reverse,80,degrees);

  // flywheel.spin(forward,6.9,voltageUnits::volt);
  // wait(0.9, sec);
  // indexer.spinFor(forward,80,degrees);
  // wait(100,msec);
  // indexer.spinFor(reverse,80,degrees);
  // flywheel.stop();
  
}

void autonomous(void) {
  enableDriver = false;
  auto3v1();
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

void usercontrol(void) {
  // User control code here, inside the loop
  enableDriver = true;
  // create tasks outside of while loop
  task toggleFlyTask(toggleFly);
  task toggleIntTask(toggleInt);
  task toggleRollTask(toggleRoll);
  while (1) {
    

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
    
    Controller1.Axis3.changed(driver);
    Controller1.Axis1.changed(driver);
    Controller1.ButtonR1.pressed(ind);
    Controller1.ButtonUp.pressed(expansion);
    
    Controller1.ButtonDown.pressed(temp);
    // Controller1.ButtonUp.pressed(driveTest2);
    // Controller1.ButtonLeft.pressed(printAngle);

    wait(100, msec);
  }
}
