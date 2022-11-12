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
      flywheel.setVelocity(100,percent);
      flywheel.spin(forward);
      enableInt = false;
    } else if(toggleF && slowF == true) {
      // if toggle on and slow
      flywheel.setVelocity(80,percent);
      flywheel.spin(forward);
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
  //indexer.setMaxTorque(100,percent);
  indexer.setVelocity(100,percent);
  //indexer.setTimeout(2, seconds);
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
void temp() {
  printf("Right Drive: %f\n", MotorGroupR.temperature(percent));
  printf("Left Drive: %f\n", MotorGroupL.temperature(percent));
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
  if(MotorGroupR.temperature(percent) >= 60) {
    Brain.Screen.print("right drive: ");
    Brain.Screen.print(MotorGroupR.temperature(percent));
    Brain.Screen.newLine();
  }
  if(MotorGroupL.temperature(percent) >= 60) {
    Brain.Screen.print("left drive: ");
    Brain.Screen.print(MotorGroupL.temperature(percent));
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
/*
void turnTo(double ang, int Vel) {
  double angD = ang - Inertial.heading(degrees);
  MotorGroupL.setVelocity(Vel,percent);
  MotorGroupR.setVelocity(Vel,percent);
  if(angD < 0) {
    MotorGroupL.spinFor(reverse,angD * TURN,degrees, false);
    MotorGroupR.spinFor(forward,angD * TURN,degrees, false);
  } else {
    MotorGroupL.spinFor(forward,angD * TURN,degrees, false);
    MotorGroupR.spinFor(reverse,angD * TURN,degrees, false);
  }
  
}
*/
void turnTo(double ang, int Vel) {
  Inertial.setHeading(0,degrees);
  MotorGroupL.setVelocity(Vel,percent);
  MotorGroupR.setVelocity(Vel,percent);
  int angD = ang - Inertial.heading(degrees);
  if(angD > 0) {
    MotorGroupL.spin(forward);
    MotorGroupR.spin(reverse);
  } else {
    MotorGroupL.spin(reverse);
    MotorGroupR.spin(forward);
  }
  waitUntil(abs(angD) <= 5);
  
  MotorGroupL.stop();
  MotorGroupR.stop();
}
void pDrive() {
  double dis = Distance.objectDistance(inches) * 1.5;
  while(Distance.objectDistance(inches) >= 8) {
    dis = Distance.objectDistance(inches) * 1.5;
    MotorGroupL.setVelocity(dis,percent);
    MotorGroupR.setVelocity(dis,percent);
    MotorGroupL.spin(forward);
    MotorGroupR.spin(forward);
  }
  MotorGroupL.setStopping(coast);
  MotorGroupR.setStopping(coast);
  MotorGroupL.stop();
  MotorGroupR.stop();
}


void printAngle() {
  printf("%f\n", Inertial.heading(degrees));
}
void driveTest() {
  MotorGroupL.setStopping(coast);
  MotorGroupR.setStopping(coast);
  // driveFor(24, 80);
  // wait(3, sec);
  // turnFor(90, 40);
  // wait(3, sec);
  // driveFor(24, 80);
  // wait(3, sec);
  // turnFor(90, 40);
  // wait(3, sec);
  // driveFor(24, 80);
  // wait(3, sec);
  // turnFor(90, 40);
  // wait(3, sec);
  // driveFor(24, 80);
  // wait(3, sec);
  // turnFor(90, 40);
  MotorGroupL.setVelocity(50,percent);
  MotorGroupR.setVelocity(50,percent);
  MotorGroupR.spinFor(reverse,100,degrees, false);
  MotorGroupL.spinFor(reverse,3010,degrees, false);
  
  wait(3, sec);
}

void driveTest2() {
  //pTurn(-90);
  //turnFor(90,-40);
  printf("%f\n", Inertial.heading(degrees));
  turnFor(Inertial.heading(degrees) - 90, 30);
}
/* --------------------------------------------------------------------------*/
// Pre-Autonomous
/* --------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  enableDriver = false;
  Inertial.calibrate();
  flywheel.setStopping(coast);
  indexer.setStopping(hold);
  MotorGroupL.setStopping(brake);
  MotorGroupL.setStopping(brake);
}
/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/

void auto2() {
  // worked on 10/11/2022
  // starts with plastic tag aligned with outer roller edge 
  //  and far edge slightly inside the tile lines
  MotorGroupL.setVelocity(60,percent);
  MotorGroupR.setVelocity(60,percent);
  MotorGroupL.spinFor(forward,250,degrees, false);
  MotorGroupR.spinFor(forward,2050,degrees, false);
  wait(1.8, sec);
  intake.setVelocity(100,percent);
  intake.spinFor(reverse,1.1,turns);
  MotorGroupL.spinFor(reverse,50,degrees, false);
  MotorGroupR.spinFor(reverse,3050,degrees, false);
  wait(1.8, sec);
  flywheel.setVelocity(93,percent);
  flywheel.spin(forward);
  wait(1.5, sec);
  indexer.setVelocity(100,percent);
  indexer.spinFor(forward,60,degrees);
  wait(150,msec);
  indexer.spinFor(reverse,60,degrees);
  wait(1.4, sec);
  indexer.spinFor(forward,60,degrees);
  wait(150,msec);
  indexer.spinFor(reverse,60,degrees);
  flywheel.stop();
  
}

void auto3() {
  // worked on 10/11/2022
  // start: facing opponent side, intake on inner edge of tile,
  //  wheels 1.5 of finger from inner edge of tile 
  MotorGroupL.setVelocity(65,percent);
  MotorGroupR.setVelocity(65,percent);
  MotorGroupR.spinFor(forward,1000,degrees, false);
  MotorGroupL.spinFor(forward,2450,degrees, false);
  wait(2, sec);
  intake.setVelocity(100,percent);
  intake.spinFor(forward,2,turns);
  MotorGroupL.setVelocity(50,percent);
  MotorGroupR.setVelocity(50,percent);
  MotorGroupR.spinFor(reverse,100,degrees, false);
  MotorGroupL.spinFor(reverse,3013,degrees, false);
  wait(2.5, sec);
  flywheel.setVelocity(100,percent);
  flywheel.spin(forward);
  wait(1.7, sec);
  indexer.setVelocity(100,percent);
  indexer.spinFor(forward,60,degrees);
  wait(150,msec);
  indexer.spinFor(reverse,60,degrees);
  wait(1.5, sec);
  indexer.spinFor(forward,60,degrees);
  wait(150,msec);
  indexer.spinFor(reverse,60,degrees);
  flywheel.stop();
  
}

void autonomous(void) {
   enableDriver = false;
   auto2();
}
/* --------------------------------------------------------------------------*/
// User Control
/* --------------------------------------------------------------------------*/
void usercontrol(void) {
  enableDriver = true;
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
    Controller1.ButtonUp.pressed(driveTest2);
    Controller1.ButtonLeft.pressed(printAngle);
    Controller1.ButtonRight.pressed(driveTest);
    
    
    wait(100, msec);
  }
}

/* --------------------------------------------------------------------------*/
// NOTES:
// remainder operation: must use int variables only - % sign 
// https://www.vexforum.com/t/programming-using-a-toggle-function/73119/8
// for prog turning while driving
//// L 400 x R 2350 = turn left 90
//// L 200 x R 1170 = turn left approx 45
//// L 250 x R 2100 = turn left 90
/* --------------------------------------------------------------------------*/