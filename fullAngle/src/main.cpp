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
// roller               digital_out   H               
// Inertial             inertial      19              
// MotorR1              motor         13              
// MotorR2              motor         12              
// MotorR3              motor         11              
// MotorL1              motor         18              
// MotorL2              motor         17              
// MotorL3              motor         16              
// EncoderR             encoder       C, D            
// EncoderL             encoder       E, F            
// flywheel             motor         2               
// angler               digital_out   A               
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
    double volt = (fDesiredVal + pid)/9;
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
    printf("pid: %f   f(V): %f   f(S): %f   i: %f\n", pid+fDesiredVal, flywheel.voltage(voltageUnits::volt), flywheel.velocity(percent), intake.velocity(percent));

    // printf("%f\n", pid + desiredVal);

    fPrevError = fError;
    task::sleep(70); // delay

  }
  
  return 1;
}

void fastIndPID() {
  fly = false;

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  // int indTime = 120; 

  // fDesiredVal = 100;
  fDesiredVal = 12;
  fkP = 0.38; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  vex::task PIDfly(flyPI);
  
  wait(0.6, sec);
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt);
  // wait(1.6, sec); // 0.5
  // enableInt = false;
  wait(0.2, sec);
  // printf("1: %f\n", intake.velocity(percent));
  
  // enableInt = false;
  // wait(0.1, sec);
  // enableInt = true;
  // intake.spin(reverse,12,voltageUnits::volt);
  wait(0.3, sec);
  // printf("2: %f\n", intake.velocity(percent));
  // enableInt = false;
  // wait(0.2, sec);
  // enableInt = true;
  // intake.spin(reverse,12,voltageUnits::volt);
  // fkP = 10;
  // fkI = 10;
  wait(0.6, sec);
  // printf("3: %f\n", intake.velocity(percent));
  enableInt = false;
  vex::task::stop(PIDfly);
  fly = true;
  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0;
  enableFlywheelPID = false;
  // enableFlyPID = false;
  // intake.stop();
  // enableInt = false;
}

void driver() {
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  if((Controller1.Axis3.position(percent) > 4 || Controller1.Axis3.position(percent) < -4))
  {
    // driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent) * 1.25, percent);
    // driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent) * 1.25, percent);
    // driveL.spin(forward);
    // driveR.spin(forward);
    double l = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * 1.15)) / 9;
    double r = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * 1.15)) / 9;
    driveL.spin(forward,l,voltageUnits::volt);
    driveR.spin(forward,r,voltageUnits::volt);
  }
  else if((Controller1.Axis1.position(percent) > 4 || Controller1.Axis1.position(percent) < -4))
  {
    // driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent) * 0.85, percent);
    // driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent) * 0.85, percent);
    // driveL.spin(forward);
    // driveR.spin(forward);
    double l = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * 1)) / 9;
    double r = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * 1)) / 9;
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

int toggleFly() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonY.pressing() && !Controller1.ButtonR2.pressing() && !lastF) { 
    // if buttonA is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = false; // set to normal speed
      fly = true;
      // enableFlyPID = false;
      enableFlywheelPID = false;
    } else if(!Controller1.ButtonY.pressing() && Controller1.ButtonR2.pressing() && !lastF) {
    // else if buttonY is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = true; // set to slower speed
      fly = true;
      // enableFlyPID = false;
      enableFlywheelPID = false;

    } else if(!Controller1.ButtonR2.pressing() && !Controller1.ButtonY.pressing()) {
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
      /*default*/////////////////////////////////////////////////
      flywheel.setStopping(coast);
      flywheel.spin(forward,11.0,voltageUnits::volt);

    } else if(toggleF && slowF) {
      // if toggle on and slow
      flywheel.setStopping(coast);
      flywheel.spin(forward,9,voltageUnits::volt);
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
  intake.spin(reverse,100,percent);
  wait(0.3, sec);
  intake.stop();
  enableInt = false;
  
}


int toggleInt() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonL2.pressing() && !Controller1.ButtonX.pressing() && !lastI) { 
    // if buttonB is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = false; // set to forward
    } else if(!Controller1.ButtonL2.pressing() && Controller1.ButtonX.pressing() && !lastI) {
    // else if buttonX is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = true; // set to reverse
    } else if(!Controller1.ButtonL2.pressing() && !Controller1.ButtonX.pressing()) {
      // else if buttons not pressing:
      lastI = false; // button was not pressed before
    }
    if(toggleI && revI == false) {
      // if toggle on and not reverse
      intVel = 95;
      intake.setVelocity(intVel,percent);
      intake.spin(forward);
      
      // flywheel.spin(forward,2,voltageUnits::volt);
    } else if((toggleI && revI == true) || enableInt) {
      // else if toggle on and reverse
      intVel = 70;
      intake.setVelocity(intVel,percent);
      intake.spin(reverse);
    } else {
      intake.stop();
      
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
int toggleAngl() {
  while(enableDriver) {
    if(Controller1.ButtonL1.pressing() && !lastA) {
      toggleA = !toggleA;
      lastA = true;
    }
    else if(!Controller1.ButtonL1.pressing()) {
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
/*
void expansion() {
  if(enableDriver) {
    expand.set(true);
    wait(2,sec);
    expand.set(false);
  }
}
*/
void temp() {
  printf("Right Drive: %f\n", driveR.temperature(percent));
  printf("Left Drive: %f\n", driveL.temperature(percent));
  printf("Flywheel: %f\n", flywheel.temperature(percent));
  printf("Intake: %f\n", intake.temperature(percent));
  
  bool overheat = false;
  Brain.Screen.clearScreen();
  if(intake.temperature(percent) >= 60) {
    Brain.Screen.print("intake: ");
    Brain.Screen.print(intake.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(flywheel.temperature(percent) >= 60) {
    Brain.Screen.print("flywheel: ");
    Brain.Screen.print(flywheel.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(driveR.temperature(percent) >= 60) {
    Brain.Screen.print("right drive: ");
    Brain.Screen.print(driveR.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(driveL.temperature(percent) >= 60) {
    Brain.Screen.print("left drive: ");
    Brain.Screen.print(driveL.temperature(percent));
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
}

/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/



/*================================================================================================*/
/* TOURNAMENT AUTONS                                                                              */
/*================================================================================================*/

void auto3v2() {
  // 2 tile side
  
  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;

  fDesiredVal = 12;
  fkP = 10; //0.38
  fkI = 10; //0.1
  fkD = 0; //0.06
  vex::task spinfly(flyPI);
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveR.spinFor(reverse,1800,degrees, false); //2150 //2225
  driveL.spinFor(reverse,475,degrees, false); //700
  wait(1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.2, sec);
  intake.stop();
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,175,degrees,false); // 170
  driveR.spinFor(forward,100,degrees,false);
  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(1.3, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.25, sec);
  intake.stop();
  wait(0.7, sec);
  intake.spin(reverse,100,percent);
  wait(0.3, sec);
  intake.stop(); 
  wait(0.2, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  /*
  // // turn to intake
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);

  driveL.spinFor(reverse,0.82,turns,false);
  driveR.spinFor(forward,0.82,turns,false);
  wait(0.5,sec);
  // intake 
  intake.setVelocity(100,percent);
  intake.spin(forward);
  driveL.setVelocity(40,percent);
  driveR.setVelocity(40,percent);
  driveL.spinFor(forward,10.5,turns,false);
  driveR.spinFor(forward,10.5,turns,false);
  wait(2.8, sec); // 2.7
  
  // turn to shoot
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveL.spinFor(forward,1.5,turns,false);
  driveR.spinFor(reverse,1.5,turns,false);
  wait(1.3, sec);
  intake.stop();
  
  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;
  // shoot 3
  fDesiredVal = 12;
  fkP = 500; //0.38
  fkI = 500; //0.1
  fkD = 0.0; //0.06

  // vex::task spinfly(flywheelPID);
  vex::task::resume(spinfly);
  intake.stop();
  wait(1.8, sec);

  intake.spin(reverse,100,percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.4, sec);
  intake.spin(reverse,100,percent);
  wait(0.2, sec);
  intake.stop(); 
  wait(0.25, sec);
  intake.spin(reverse,100,percent);
  wait(0.2, sec);
  intake.stop(); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  */
}


void autonomous(void) {
  enableDriver = false;
  auto3v2();

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
  intVel = 90;
  // angler.set(true);
  // create tasks outside of while loop
  task toggleFlyTask(toggleFly);
  task toggleIntTask(toggleInt);
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
    
    Controller1.Axis3.changed(driver);
    Controller1.Axis1.changed(driver);
    Controller1.ButtonA.pressed(ind);
    Controller1.ButtonR1.pressed(fastIndPID);
    // Controller1.ButtonUp.pressed(expansion);
    
    Controller1.ButtonDown.pressed(temp);

    wait(100, msec);
  }
}
