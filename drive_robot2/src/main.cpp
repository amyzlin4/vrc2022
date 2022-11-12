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
// Inertial             inertial      19              
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
double TURN = 8;
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
// void intaker() {

//   intake.setVelocity(100,percent);
//   intake.spin(forward);

// }
// void stopIntaker() {
//   intake.stop();
// }
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
  Inertial.calibrate();
}

/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/

 void printStuff() {
   printf("%f\n", Inertial.heading(degrees));
  //  Brain.Screen.print(Inertial.heading(degrees));
  //  Brain.Screen.newLine();
 }

 void turnUntil(int ang, int vel) {
   int diff = Inertial.heading(degrees) - ang;
   driveL.setVelocity(vel,percent);
   driveR.setVelocity(vel,percent);
   if(abs(diff) > 5) {
     if(diff > 0) {
        driveL.spinFor(forward,diff * TURN, degrees, false);
        driveR.spinFor(reverse,diff * TURN, degrees, false);

     } else if (diff < 0) {
        driveL.spinFor(reverse,diff * TURN, degrees, false);
        driveR.spinFor(forward,diff * TURN, degrees, false);
     }
     
   }
 }

void turnTo(double ang, int Vel) {
  // Inertial.setHeading(0,degrees);
  // wait(0.2, sec);
  driveL.setVelocity(Vel,percent);
  driveR.setVelocity(Vel,percent);
  int angD = ang - Inertial.heading(degrees);
  if(angD > 0) {
    driveL.spin(forward);
    driveR.spin(reverse);
  } else {
    driveL.spin(reverse);
    driveR.spin(forward);
  }
  waitUntil(abs(angD) <= 10);
  
  driveL.stop();
  driveR.stop();
}

void turnFor(double ang, int vel) {
  // Inertial.calibrate();
  // wait(0.5,sec);
  driveL.setVelocity(vel,percent);
  driveR.setVelocity(vel,percent);
  driveL.spinFor(forward,ang * TURN,degrees, false);
  driveR.spinFor(reverse,ang * TURN,degrees, false);
  printf("%f\n", Inertial.heading(degrees));
}

void pDrive() {
  double dis = Distance.objectDistance(inches) * 1.5;
  while(Distance.objectDistance(inches) >= 8) {
    dis = Distance.objectDistance(inches) * 1.5;
    driveL.setVelocity(dis,percent);
    driveR.setVelocity(dis,percent);
    driveL.spin(forward);
    driveR.spin(forward);
  }
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  driveL.stop();
  driveR.stop();
}

void goTo(double x, double y) {
  double bx = x - GPS.xPosition(inches);
  double cy = y - GPS.yPosition(inches);

  if(bx != 0 && cy != 0)
  {
    double aDis = sqrt(bx * bx + cy * cy);
    double ang = asin(bx / aDis);

    if(cy < 0) {
      ang = 180 - ang;
    }
    if(bkwd == -1) {
      ang = ang + 180;
    }
    if(ang >= 360) {
      ang = ang - 360;
    }

    // turn to heading ang degrees

    // stop driving
    driveL.stop();
    driveR.stop();

    // drive for aDis * driveConstant * bkwd inches
  }
}

 void test() {
  //  printf("x: %f\n", GPS.xPosition(inches));
  //  printf("y: %f\n", GPS.yPosition(inches));
  //  printf("heading: %f\n", GPS.heading());
  turnFor(90, 20);
  wait(2, sec);
  printf("%f\n", Inertial.heading(degrees));

 }


void auto2v1() {
  // 3 tile side 
  flywheel.spin(forward,7.7,voltageUnits::volt);
  roller.set(true);
  wait(0.5, sec);
  driveL.setVelocity(40,percent);
  driveR.setVelocity(40,percent);
  driveL.spinFor(forward,110,degrees,false);
  driveR.spinFor(forward,110,degrees,false);
  wait(0.8, sec);
  roller.set(false);
  wait(0.5, sec);
  
  driveR.setVelocity(20,percent);
  driveR.spinFor(forward,65,degrees);
  wait(0.7, sec);

  
  wait(1, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.spin(forward,8.1,voltageUnits::volt);
  wait(2.5, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.stop();
  
}

void auto2v2() {
  // 3 tile side 
  roller.set(true);
  wait(0.5, sec);
  driveL.setVelocity(40,percent);
  driveR.setVelocity(40,percent);
  driveL.spinFor(forward,100,degrees,false);
  driveR.spinFor(forward,100,degrees,false);
  wait(0.8, sec);
  roller.set(false);
  wait(0.5, sec);
  driveR.setVelocity(20,percent);
  driveR.spinFor(forward,75,degrees);
  wait(0.9, sec);
  // shoot 2
  flywheel.spin(forward,8.12,voltageUnits::volt);
  wait(1.5, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.spin(forward,8.1,voltageUnits::volt);
  wait(0.9, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.stop();
  // turn to intake
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveL.spinFor(forward,1.08,turns,false);
  driveR.spinFor(reverse,1.08,turns,false);
  wait(1, sec);
  // intake 3
  intake.setVelocity(100,percent);
  intake.spin(forward);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,6,turns,false);
  driveR.spinFor(forward,6,turns,false);
  wait(2, sec);
  driveL.setVelocity(10,percent);
  driveR.setVelocity(10,percent);
  driveL.spinFor(forward,6,turns,false);
  driveR.spinFor(forward,6,turns,false);
  wait(5.5,sec);
  // turn to shoot
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  intake.stop();
  flywheel.spin(forward,6.7,voltageUnits::volt);
  driveL.spinFor(reverse,1.7,turns,false);
  driveR.spinFor(forward,1.7,turns,false);
  wait(2, sec);
  // shoot 3
  
  wait(0.5, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);

  flywheel.spin(forward,6.84,voltageUnits::volt);
  wait(0.9, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);

  flywheel.spin(forward,6.9,voltageUnits::volt);
  wait(0.9, sec);
  indexer.spinFor(forward,80,degrees);
  wait(100,msec);
  indexer.spinFor(reverse,80,degrees);
  flywheel.stop();
  
}

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
  // flywheel.spin(forward,7.66,voltageUnits::volt);
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
  // wait(0.9, sec);
 
  // wait(1.5, sec);
  // indexer.spinFor(forward,80,degrees);
  // wait(100,msec);
  // indexer.spinFor(reverse,80,degrees);
  // flywheel.spin(forward,7.9,voltageUnits::volt);
  // wait(2.5, sec);
  // indexer.spinFor(forward,80,degrees);
  // wait(100,msec);
  // indexer.spinFor(reverse,80,degrees);
  // flywheel.stop();

  // turn to intake
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveL.spinFor(reverse,1.2,turns,false);
  driveR.spinFor(forward,1.2,turns,false);
  wait(1.2, sec);
  // intake 1
  // intake.setVelocity(100,percent);
  // intake.spin(forward);
  driveL.setVelocity(75,percent);
  driveR.setVelocity(75,percent);
  // flywheel.spin(reverse,1,voltageUnits::volt);
  driveL.spinFor(forward,18,turns,false);
  driveR.spinFor(forward,18,turns,false);
  wait(3.5, sec);
  // 180
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveL.spinFor(reverse,4,turns,false);
  driveR.spinFor(forward,4,turns,false);
  wait(1.5, sec);
  driveL.spinFor(reverse,9,turns,false);
  driveR.spinFor(reverse,9,turns,false);
  wait(2, sec);
  driveL.spinFor(reverse,1.1,turns,false);
  driveR.spinFor(forward,1.1,turns,false);
  wait(1, sec);
 
  
}


void auto3v3() {
  // 2 tile side
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  driveL.setVelocity(55,percent);
  driveR.setVelocity(55,percent);
  driveR.spinFor(reverse,2450,degrees, false);
  driveL.spinFor(reverse,1000,degrees, false);
  wait(1.8, sec);
  // flywheel.spin(forward,7.66,voltageUnits::volt);
  roller.set(true);
  wait(0.3, sec);
  driveL.setVelocity(40,percent);
  driveR.setVelocity(40,percent);
  driveL.spinFor(forward,110,degrees,false);
  driveR.spinFor(forward,110,degrees,false);
  wait(0.5, sec);
  roller.set(false);
  wait(0.3, sec);
  driveL.spinFor(forward,200,degrees);
  wait(0.25, sec);
  // turn to intake
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveL.spinFor(reverse,1.2,turns,false);
  driveR.spinFor(forward,1.2,turns,false);
  wait(0.8, sec);
  // intake 
  intake.setVelocity(100,percent);
  intake.spin(forward);
  driveL.setVelocity(75,percent);
  driveR.setVelocity(75,percent);
  flywheel.spin(reverse,1,voltageUnits::volt);
  driveL.spinFor(forward,16.9,turns,false);
  driveR.spinFor(forward,16.9,turns,false);
  wait(2.4, sec);
  intake.stop();
  flywheel.stop();
  // 180
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(reverse,4.11,turns,false);
  driveR.spinFor(forward,4.11,turns,false);
  wait(1.5, sec);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(reverse,8.7,turns,false);
  driveR.spinFor(reverse,8.7,turns,false);
  wait(2, sec);
  driveL.spinFor(reverse,1.1,turns,false);
  driveR.spinFor(forward,1.1,turns,false);
  wait(0.9, sec);
  // roll
  driveL.setVelocity(70,percent);
  driveR.setVelocity(70,percent);
  driveL.spinFor(reverse,2.5,turns,false);
  driveR.spinFor(reverse,2.5,turns,false);
  wait(1.5, sec);
  roller.set(true);
  wait(0.3, sec);
  driveL.setVelocity(40,percent);
  driveR.setVelocity(40,percent);
  driveL.spinFor(forward,210,degrees,false);
  driveR.spinFor(forward,210,degrees,false);
  wait(0.5, sec);
  roller.set(false);
  wait(0.3, sec);
}

void autonomous(void) {
  enableDriver = false;
  auto3v3();
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
    Controller1.ButtonRight.pressed(test);
    Controller1.ButtonLeft.pressed(printStuff);

    wait(100, msec);
  }
}
