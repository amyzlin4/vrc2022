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
// intake               motor         3               
// Controller1          controller                    
// Inertial             inertial      1               
// MotorR1              motor         8               
// MotorR2              motor         9               
// MotorR3              motor         10              
// flywheel             motor         2               
// angler               digital_out   A               
// expand               digital_out   B               
// MotorL1              motor         11              
// MotorL2              motor         12              
// MotorL3              motor         13              
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
double DRIVE = 0.014; // update
double TURN = 8; // update
bool lastP = false;
bool toggleP = false;
bool enableDriver = true;
bool enableInt = true;
bool toggleR = false;
bool lastR = false;
int bkwd = 1;
bool fly = true;
bool toggleA = false;
bool lastA = false;
// constants
// double kp = 0.15; // decreases as voltage increases
// double ki = 0.9; // tends to not change
// double kd = 0.03; // decreases for second and third discs
// double error;
// double prevError = 0;
// double deriv;
// double totalError = 0;
// double desiredVal = 0;
bool enableFlyPID = false;
/*===================================================================*/
double fkP = 0.15; // decreases as voltage increases
double fkI = 0.9; // tends to not change
double fkD = 0.03; // decreases for second and third discs
double fDeriv;
double fDesiredVal;
double fDerivative;
double fError;
double fPrevError = 0;
double fTotalError = 0;
double totalTimesShot = 0;
/*===================================================================*/
double dkP = 0.0;
double dkI = 0.0;
double dkD = 0.0;
double dDeriv;
double dDesiredVal;
double dErr;
double dPrevErr = 0;
double dTotalErr = 0;
bool enableDrivePID = false;

// DRIVETRAIN:
// double wheelTravel = 12.56; // circumference 
double trackWidth = 1.0; // front edge, wheel to wheel
// double trackLength = 12; // side edge, axle to axle
motor_group driveL = motor_group(MotorL1, MotorL2, MotorL3);
motor_group driveR = motor_group(MotorR1, MotorR2, MotorR3);

// DRIVER CONTROL FUNCTIONS:
/*================================================================================================*/

int flyPI() {

  while(true) {

    // proportional:
    fError = fDesiredVal - flywheel.voltage(voltageUnits::volt);

    // derivative:
    fDeriv = fError - fPrevError;

    // integral: 
    fTotalError += fError;

    double pid = fError * fkP + fDeriv * fkD + fTotalError * fkI;
    flywheel.spin(forward, pid + fDesiredVal, voltageUnits::volt);
    printf("pid: %f   f(V): %f   f(S): %f   i: %f\n", pid+fDesiredVal, flywheel.voltage(voltageUnits::volt), flywheel.velocity(percent), intake.velocity(percent));
    // printf("%f\n", pid + fDesiredVal);

    fPrevError = fError;
    task::sleep(50); // delay
  }

  return 1;
}

////////////////////////////////////////////////////////////////////////////////////
bool enableFlywheelPID = false;

/*

int flywheelPID(){

  while(true){
    // enableFlywheelPID = true;
    // enableFlyPID = true;
    double velocity = flywheel.velocity(percent);
  
    //printf("velo: %f\n", velocity);
    // double velo = flywheel.velocity(percent);
    // proportional:
    fError = fDesiredVal - velocity;
    
    // derivative:
    fDerivative = fError - fPrevError;

    // integral: 
    fTotalError += fError;

    double pid = fError * fkP + fDerivative * fkD + fTotalError * fkI;
    //printf("E: %f, D: %f, TE: %f, pid: %f\n", fError, fDeriv, fTotalError, pid);
    double volt = (fDesiredVal + pid)/7;
    if (volt > 12){
      // printf("volt > 10: %f\n", volt);
      volt = 12;
      
    }else if(volt < 1){
      // printf("volt < 1: %f\n", volt);
      volt = 1;
    }
    flywheel.spin(forward, volt, voltageUnits::volt);
    
    //flywheel.spin(forward, 50, percentUnits::pct);
    // printf("err: %f pid:%f cvelo: %f cvolt: %f %f\n",velocity, fError, pid, volt, totalTimesShot);
    printf("pid: %f   f(V): %f   f(RPM): %f   f(PCT): %f   i: %f\n", volt, flywheel.voltage(voltageUnits::volt), flywheel.velocity(rpm), flywheel.velocity(percent), intake.velocity(percent));

    // printf("%f\n", pid + desiredVal);

    fPrevError = fError;
    task::sleep(30); // delay

  }
  
  return 1;
}
*/

void driver() {
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  if((Controller1.Axis3.position(percent) > 1 || Controller1.Axis3.position(percent) < -1))
  {
    // driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent) * 1.25, percent);
    // driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent) * 1.25, percent);
    // driveL.spin(forward);
    // driveR.spin(forward);
    double l = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * 0.25)) / 9;
    double r = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * 0.25)) / 9;
    driveL.spin(forward,l,voltageUnits::volt);
    driveR.spin(forward,r,voltageUnits::volt);
  }
  else if((Controller1.Axis1.position(percent) > 1 || Controller1.Axis1.position(percent) < -1))
  {
    // driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent) * 0.85, percent);
    // driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent) * 0.85, percent);
    // driveL.spin(forward);
    // driveR.spin(forward);
    double l = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * 0.8)) / 9;
    double r = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * 0.8)) / 9;
    driveL.spin(forward,l,voltageUnits::volt);
    driveR.spin(forward,r,voltageUnits::volt);
  }
  else
  {
    driveL.setVelocity(0,percent);
    driveR.setVelocity(0,percent);
    driveL.spin(forward,0,voltageUnits::volt);
    driveR.spin(forward,0,voltageUnits::volt);
  }
  
}

void arcade() {
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  double constant = 12/127;
  double l1 = (Controller1.Axis3.value() + Controller1.Axis1.value() + Controller1.Axis4.value()) * constant;
  double l2 = (Controller1.Axis3.value() + Controller1.Axis1.value() + Controller1.Axis4.value()) * constant;
  double l3 = (Controller1.Axis3.value() + Controller1.Axis1.value() - Controller1.Axis4.value()) * constant;
  double r1 = (Controller1.Axis3.value() - Controller1.Axis1.value() - Controller1.Axis4.value()) * constant;
  double r2 = (Controller1.Axis3.value() - Controller1.Axis1.value() - Controller1.Axis4.value()) * constant;
  double r3 = (Controller1.Axis3.value() - Controller1.Axis1.value() + Controller1.Axis4.value()) * constant;
  MotorL1.spin(forward, l1, voltageUnits::volt);
  MotorL2.spin(forward, l2, voltageUnits::volt);
  MotorL3.spin(forward, l3, voltageUnits::volt);
  MotorR1.spin(forward, r1, voltageUnits::volt);
  MotorR2.spin(forward, r2, voltageUnits::volt);
  MotorR3.spin(forward, r3, voltageUnits::volt);
}


int toggleFly() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonA.pressing() && !Controller1.ButtonY.pressing() && !lastF) { 
    // if buttonY is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = false; // set to normal speed
      fly = true;
      // enableFlyPID = false;
      enableFlywheelPID = false;
    } else if(!Controller1.ButtonA.pressing() && Controller1.ButtonY.pressing() && !lastF) {
    // else if buttonB is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = true; // set to slower speed
      fly = true;
      // enableFlyPID = false;
      enableFlywheelPID = false;

    } else if(!Controller1.ButtonY.pressing() && !Controller1.ButtonA.pressing()) {
    // else if button is not pressing:
      lastF = false; // button was not pressed before
      ///////// try this:
      // enableFlyPID = false;
      enableFlywheelPID = false; // remove?
      // fly = true;
      // flywheel.spin(forward,1.5,voltageUnits::volt);
    }
    
    if(toggleF && !slowF) {
      // if toggle on and not slow
      flywheel.setStopping(coast);
      flywheel.spin(forward,12.0,voltageUnits::volt);

    } else if(toggleF && slowF) {
      // if toggle on and slow
      flywheel.setStopping(coast);
      flywheel.spin(forward,10,voltageUnits::volt);
    } else if(fly && !enableFlywheelPID) {
      
      flywheel.spin(forward,4.5,voltageUnits::volt);
      // flywheel.setVelocity(0,percent);
      // flywheel.stop();

    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}

void ind() {
  enableInt = true;
  intake.spin(reverse,10,voltageUnits::volt);
  wait(0.2, sec);
  intake.stop();
  enableInt = false;
  
}


int toggleInt() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonL1.pressing() && !Controller1.ButtonR1.pressing() && !lastI) { 
    // if buttonB is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = false; // set to forward
    } else if(!Controller1.ButtonL1.pressing() && Controller1.ButtonR1.pressing() && !lastI) {
    // else if buttonX is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = true; // set to reverse
    } else if(!Controller1.ButtonL1.pressing() && !Controller1.ButtonR1.pressing()) {
      // else if buttons not pressing:
      lastI = false; // button was not pressed before
    }
    if(toggleI && revI == false) {
      // if toggle on and not reverse
      intVel = 100;
      intake.setVelocity(intVel,percent);
      intake.spin(forward);
      
      // flywheel.spin(forward,2,voltageUnits::volt);
    } else if((toggleI && revI == true) || enableInt) {
      // else if toggle on and reverse
      intVel = 70;
      intake.setVelocity(intVel,percent);
      intake.spin(reverse);
    } else if(!enableFlywheelPID){
      intake.stop();
      
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}

void tripShot() {
  fly = false;

  driveL.setStopping(hold);
  driveR.setStopping(hold);

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  // fDesiredVal = 100;
  // fDesiredVal = 12;
  // fkP = 0.38; //0.38
  // fkI = 0; //0.1
  // fkD = 0.1; //0.06
  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  
  wait(0.4, sec); 
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt);
 
  wait(0.15, sec);
  
  enableInt = false;
  wait(0.18, sec);
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt);
  wait(0.16, sec);

  enableInt = false;
  wait(0.15, sec);
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt);

  wait(0.3, sec);
  enableInt = false;
  // vex::task::stop(PIDfly);
  flywheel.setStopping(coast);
  flywheel.stop();
  intake.stop();
  fly = true;
  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0;
  enableFlywheelPID = false;
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  enableInt = false;

}

int inta() {
  while(enableDriver) {
    if(Controller1.ButtonL1.pressing()) {
      intake.spin(forward,100,percent);
    }
    else if(Controller1.ButtonR1.pressing() || enableInt) {
      intake.spin(reverse,95, percent);
    }
    else if(!enableFlywheelPID){
      intake.stop();
    }
  }
  return 1;
}

int toggleAngl() {
  while(enableDriver) {
    if(Controller1.ButtonL2.pressing() && !lastA) {
      toggleA = !toggleA;
      lastA = true;
    }
    else if(!Controller1.ButtonL2.pressing()) {
      lastA = false;
    }
    if(toggleA) {
      angler.set(true);
    }
    else {
      angler.set(false);
    }
  }
  return 1;
}

void expansion() {
  if(Controller1.ButtonUp.pressing()) {
    expand.set(true);
    wait(2,sec);
    expand.set(false);
  }
}


void temp() {
  printf("Right Drive: %f\n", driveR.temperature(percent));
  printf("Left Drive: %f\n", driveL.temperature(percent));
  printf("Flywheel: %f\n", flywheel.temperature(percent));
  printf("Intake: %f\n", intake.temperature(percent));

  double temp = 60;
  
  bool overheat = false;
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(0,0);
  if(intake.temperature(percent) >= temp) {
    Brain.Screen.print("intake: ");
    Brain.Screen.print(intake.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(flywheel.temperature(percent) >= temp) {
    Brain.Screen.print("flywheel: ");
    Brain.Screen.print(flywheel.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(MotorR1.temperature(percent) >= temp) {
    Brain.Screen.print("R1: ");
    Brain.Screen.print(MotorR1.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(MotorR2.temperature(percent) >= temp) {
    Brain.Screen.print("R2: ");
    Brain.Screen.print(MotorR2.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(MotorR3.temperature(percent) >= temp) {
    Brain.Screen.print("R3: ");
    Brain.Screen.print(MotorR3.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(MotorL1.temperature(percent) >= temp) {
    Brain.Screen.print("L1: ");
    Brain.Screen.print(MotorL1.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(MotorL2.temperature(percent) >= temp) {
    Brain.Screen.print("L2: ");
    Brain.Screen.print(MotorL2.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(MotorL3.temperature(percent) >= temp) {
    Brain.Screen.print("L3: ");
    Brain.Screen.print(MotorL3.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(!overheat) {
    Brain.Screen.print("Nothing overheated... yet :)");
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
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  Inertial.calibrate();
  angler.set(false);
  expand.set(false);
}

/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/

//TEST CODE
void testFwd() {
  // driveL.setVelocity(70,percent);
  // driveR.setVelocity(70,percent);
  // driveR.spinFor(forward, 10, turns, false);
  // driveL.spinFor(forward, 10, turns, false);
  
  driveL.spin(forward,7,voltageUnits::volt);
  driveR.spin(forward,7,voltageUnits::volt);

  wait(1.5, sec); //1.5
  driveL.stop();
  driveR.stop();
}
void testBwd() {
  // driveL.setVelocity(70,percent);
  // driveR.setVelocity(70,percent);
  // driveL.spinFor(reverse, 10, turns, false);
  // driveR.spinFor(reverse, 10, turns, false);

  driveL.spin(reverse,7,voltageUnits::volt);
  driveR.spin(reverse,7,voltageUnits::volt);

  wait(1.5 , sec); //1.5
  driveL.stop();
  driveR.stop();
}

void setCoast() {
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  Controller1.Screen.clearScreen();
  Controller1.Screen.print("COAST");
}
void setBrake() {
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  Controller1.Screen.clearScreen();
  Controller1.Screen.print("BRAKE");
}
void setHold() {
  driveL.setStopping(hold);
  driveR.setStopping(hold);
  Controller1.Screen.clearScreen();
  Controller1.Screen.print("HOLD");
}

/*================================================================================================*/
/* TOURNAMENT AUTONS                                                                              */
/*================================================================================================*/


void auto2v1() {
  // 3 tile side 
  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;

  fDesiredVal = 12;
  fkP = 0.48; //0.38
  fkI = 0.1; //0.1
  fkD = 0.0; //0.06

  vex::task spinfly(flyPI);
  
  intake.spin(forward,100,percent);
  driveL.spin(reverse,10,percent);
  driveR.spin(reverse,10,percent);
  wait(0.4, sec);
  intake.stop();
  driveL.stop();
  driveR.stop();
  wait(0.3, sec);
  driveL.setVelocity(20,percent);
  driveR.setVelocity(20,percent);
  driveL.spinFor(forward,200,degrees,false);
  driveR.spinFor(forward,275,degrees,false);

  wait(1.35, sec);

  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.2, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.3, sec);

  intake.spin(reverse,100,percent);
  wait(0.3, sec);
  intake.stop(); 
  wait(0.1, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  
  
}



void autonomous(void) {
  // enableDriver = false;
  // auto2v1();
  testFwd();

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
  enableInt = false;
  // intVel = 90;
  // angler.set(true);
  // create tasks outside of while loop
  task toggleFlyTask(toggleFly);
  task toggleIntaTask(inta);
  task toggleAnglTask(toggleAngl);
  task::setPriority(toggleFlyTask,2);
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
    
    Controller1.Axis3.changed(arcade);
    Controller1.Axis1.changed(arcade);
    Controller1.Axis4.changed(arcade);
    // Controller1.ButtonB.pressed(ind);
    // Controller1.ButtonR2.pressed(tripShot);
    // Controller1.ButtonR2.pressed(tripShot2);
    // Controller1.ButtonUp.pressed(expansion);
    // Controller1.ButtonB.pressed(farShot);

    // Controller1.ButtonB.pressed(setCoast);
    // Controller1.ButtonY.pressed(setBrake);
    // Controller1.ButtonX.pressed(setHold);

    // Controller1.ButtonR1.pressed(testFwd);
    // Controller1.ButtonR2.pressed(testBwd);
    
    Controller1.ButtonDown.pressed(temp);

    wait(100, msec);
  }
}
