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
// driveL               motor_group   17, 13          
// flywheel             motor_group   4, 6            
// intake               motor         3               
// indexer              motor         7               
// Controller1          controller                    
// roller               digital_out   H               
// expand               digital_out   A               
// Inertial             inertial      16              
// Distance             distance      10              
// GPS                  gps           1               
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
int bkwd = 1;
bool fly = true;

// DRIVER CONTROL FUNCTIONS:
void driver() {
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  if((Controller1.Axis3.position(percent) > 4 || Controller1.Axis3.position(percent) < -4))
  {
    driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent), percent);
    driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent), percent);
    driveL.spin(forward);
    driveR.spin(forward);
  }
  else if(Controller1.Axis1.position(percent) != 0)
  {
    driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent), percent);
    driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent), percent);
    driveL.spin(forward);
    driveR.spin(forward);
  }
  else
  {
    driveL.setVelocity(0,percent);
    driveR.setVelocity(0,percent);
  }
  
}
int toggleFly() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonY.pressing() == true && Controller1.ButtonA.pressing() == false && !lastF) { 
    // if buttonA is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = false; // set to normal speed
      fly = true;
    } else if(Controller1.ButtonY.pressing() == false && Controller1.ButtonA.pressing() == true && !lastF) {
    // else if buttonY is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = true; // set to slower speed
      fly = true;
    } else if(Controller1.ButtonA.pressing() == false && Controller1.ButtonY.pressing() == false) {
    // else if button is not pressing:
      lastF = false; // button was not pressed before
      enableInt = true;
    }
    if(toggleF && slowF == false) {
      // if toggle on and not slow
      
      flywheel.spin(forward,9.0,voltageUnits::volt);
      // flywheel.spin(forward,10.0,voltageUnits::volt);
      // wait(1, sec);
      // flywheel.spin(forward,8.0,voltageUnits::volt);
      enableInt = false;
    } else if(toggleF && slowF == true) {
      // if toggle on and slow
      // flywheel.setVelocity(65,percent);
      // flywheel.spin(forward);
      // flywheel.spin(forward,10.0,voltageUnits::volt);
      // wait(1, sec);
      flywheel.spin(forward,7.6,voltageUnits::volt);
      enableInt = false;
    } else if(fly) {
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
void fastInd() {
  fly = false;
  indexer.setStopping(hold);
  int indTime = 200;

  flywheel.spin(forward, 9.6, voltageUnits::volt);
  wait(0.4, sec);

  indexer.setPosition(0,degrees);

  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec); 

  flywheel.spin(forward, 9, voltageUnits::volt);
  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec);

  
  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec);

  indexer.spinToPosition(0,degrees);
  
  fly = true;
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
  Inertial.calibrate();
}

/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/


void autonomous(void) {
  enableDriver = false;
  
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
    Controller1.ButtonR2.pressed(fastInd);
    Controller1.ButtonUp.pressed(expansion);
    
    Controller1.ButtonDown.pressed(temp);

    wait(100, msec);
  }
}
