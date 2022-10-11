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
// Distance             distance      15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "cmath"
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
bool lastP = false;
bool toggleP = false;

/* --------------------------------------------------------------------------*/ 
// DRIVER CONTROL FUNCTIONS:
/* --------------------------------------------------------------------------*/
void driver() {
  MotorGroupL.setStopping(brake);
  MotorGroupR.setStopping(brake);
  MotorGroupL.setVelocity(Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
  MotorGroupR.setVelocity(Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
  MotorGroupL.spin(forward);
  MotorGroupR.spin(forward);
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
      flywheel.setVelocity(100,percent);
      flywheel.spin(forward);
    } else {
      flywheel.stop();
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
void ind() {
  //indexer.setMaxTorque(100,percent);
  indexer.setVelocity(100,percent);
  indexer.spinFor(forward,60,degrees);
  wait(150,msec);
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
      intVel = 90;
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
  if(indexer.temperature(percent) >= 60) {
    Brain.Screen.print(indexer.temperature(percent));
    Brain.Screen.newLine();
  }
  if(intake.temperature(percent) >= 60) {
    Brain.Screen.print(intake.temperature(percent));
    Brain.Screen.newLine();
  }
}
void printVel() {
  printf("%f\n", flywheel.velocity(percent));
}
int togglePrint() {  // use int for tasks (?)
  while(true) {
    if(Controller1.ButtonRight.pressing() == true && !lastP) { 
    // if button is pressing and button was not pressed before:
      toggleP = !toggleP;
      lastP = true; // button was pressed before
    } else if(Controller1.ButtonRight.pressing() == false) {
    // else if button is not pressing:
      lastP = false;
    }
    if(toggleP) {
      printf("%f\n", flywheel.velocity(percent));
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
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
  MotorGroupL.setStopping(brake);
  MotorGroupL.setStopping(brake);
}
/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/
void auto1() {
  
}

void autonomous(void) {
  
}
/* --------------------------------------------------------------------------*/
// User Control
/* --------------------------------------------------------------------------*/
void usercontrol(void) {
  // create tasks outside of while loop
  task toggleFlyTask(toggleFly);
  task toggleIntTask(toggleInt);
  //task togglePrintTask(togglePrint);
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
    Controller1.Axis3.changed(driver);
    Controller1.Axis1.changed(driver);
    Controller1.ButtonR1.pressed(ind);

    Controller1.ButtonDown.pressed(temp);
    
    wait(100, msec);
  }
}
