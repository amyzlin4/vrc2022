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
// driveL               motor_group   17, 16          
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
// constants
double kp = 0.15; // decreases as voltage increases
double ki = 0.9; // tends to not change
double kd = 0.03; // decreases for second and third discs
double error;
double prevError = 0;
double deriv;
double totalError = 0;
double desiredVal = 0;
bool enableFlyPID = false;

// drivetrain
// double wheelTravel = 12.56; // circumference 
double trackWidth = 13.5; // front edge, wheel to wheel
// double trackLength = 12; // side edge, axle to axle
// smartdrive robot(driveL, driveR, Inertial, wheelTravel, trackWidth, trackLength, distanceUnits::in);

// DRIVER CONTROL FUNCTIONS:

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
int shootdiscs(){
  int indTime = 180;

  indexer.setPosition(0,degrees);
  fkD = 0.0;
  indexer.spin(forward, 9, voltageUnits::volt);
  wait(indTime - 20, msec);
  indexer.spin(reverse, 9, voltageUnits::volt);
  wait(indTime - 20, msec); 
  totalTimesShot += 1;

  // desiredVal = 6;
  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec);
  totalTimesShot += 1;

  // kd = 0.01;
  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec);
  totalTimesShot += 1;
  indexer.spinToPosition(0, degrees);

  return 1;
}

void fastInd1() {
  fly = false;
  flywheel.spin(forward, 8, voltageUnits::volt);
  wait(0.5, sec);
  vex::task shoot8(shootdiscs);
  wait(1, sec);
  flywheel.stop();
  flywheel.spin(forward, 1, voltageUnits::volt);
  fly = true;
}

bool enableFlywheelPID = false;

int flywheelPID(){
  
  // fkP = 0.28;
  // fkD = 0.03;
  // fkI = 0.9;
  // double fDerivative;
  // fError= 0;
  // double fDesiredVal = 30;

  while(enableFlywheelPID){
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
    if (volt > 8){
      printf("volt > 10: %f\n", volt);
      volt = 8;
      
    }else if(volt < 1){
      printf("volt < 1: %f\n", volt);
      volt = 1;
    }
    flywheel.spin(forward, volt, voltageUnits::volt);
    
    //flywheel.spin(forward, 50, percentUnits::pct);
    printf("err: %f pid:%f cvelo: %f cvolt: %f %f\n",velocity, fError, pid, volt, totalTimesShot);
    // printf("%f\n", pid + desiredVal);

    fPrevError = fError;
    task::sleep(70); // delay

  }
  
  return 1;
}

void fastIndPID() {
  fly = false;

  // reset
  error = 0;
  prevError = 0;
  fDerivative = 0;
  totalError = 0;

  indexer.setStopping(hold);
  int indTime = 120; //200 206

  fDesiredVal = 46;
  fkP = 0.30; //0.38
  fkI = 0.07; //0.1
  fkD = 0.03; //0.06
  
  enableFlywheelPID = true;
  vex::task PIDfly(flywheelPID);

  wait(0.55, sec);

  indexer.setPosition(0,degrees);
  fkD = 0.0;
  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec); 
  totalTimesShot += 1;
  indexer.stop();
  wait(60,msec);

  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec);
  totalTimesShot += 1;
  indexer.stop();
  wait(60,msec);

  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime-20, msec);
  totalTimesShot += 1;
  indexer.stop();

  indexer.spinToPosition(0,degrees);

  vex::task::stop(PIDfly);
  fly = true;
  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0;
  enableFlywheelPID = false;
  enableFlyPID = false;
}

void driver() {
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  if((Controller1.Axis3.position(percent) > 4 || Controller1.Axis3.position(percent) < -4))
  {
    driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent) * 1.25, percent);
    driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent) * 1.25, percent);
    driveL.spin(forward);
    driveR.spin(forward);
  }
  else if(Controller1.Axis1.position(percent) != 0)
  {
    driveL.setVelocity((Controller1.Axis3.position(percent)) + Controller1.Axis1.position(percent) * 0.85, percent);
    driveR.setVelocity((Controller1.Axis3.position(percent)) - Controller1.Axis1.position(percent) * 0.85, percent);
    driveL.spin(forward);
    driveR.spin(forward);
  }
  else
  {
    driveL.setVelocity(0,percent);
    driveR.setVelocity(0,percent);
  }
  
}
/////////////////////////////////////////////////////////////////////////////////
int flyPI() {

  while(true) {

    // proportional:
    error = desiredVal - flywheel.voltage(voltageUnits::volt);

    // derivative:
    deriv = error - prevError;

    // integral: 
    totalError += error;

    double pid = error * kp + deriv * kd + totalError * ki;
    flywheel.spin(forward, pid + desiredVal, voltageUnits::volt);
    printf("%f\n", flywheel.voltage(voltageUnits::volt));
    printf("indexer: %f\n", indexer.voltage(voltageUnits::volt));
    // printf("%f\n", pid + desiredVal);

    prevError = error;
    task::sleep(100); // delay
  }

  return 1;
}
////////////////////////////////////////////////////////////////////////////////////
int toggleFly() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonY.pressing() == true && Controller1.ButtonR2.pressing() == false && !lastF) { 
    // if buttonA is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = false; // set to normal speed
      fly = true;
      // enableFlyPID = false;
      enableFlywheelPID = false;
    } else if(Controller1.ButtonY.pressing() == false && Controller1.ButtonR2.pressing() == true && !lastF) {
    // else if buttonY is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = true; // set to slower speed
      fly = true;
      // enableFlyPID = false;
      enableFlywheelPID = false;

    } else if(Controller1.ButtonR2.pressing() == false && Controller1.ButtonY.pressing() == false ) {
    // else if button is not pressing:
      lastF = false; // button was not pressed before
      enableInt = true;
      ///////// try this:
      // enableFlyPID = false;
      enableFlywheelPID = false;
      // fly = true;
      // flywheel.spin(forward,1.5,voltageUnits::volt);
    }
    // double diff = 0;
    // double goal = 0;
    if(toggleF && slowF == false) {
      // if toggle on and not slow
      /*default*/////////////////////////////////////////////////
      flywheel.setStopping(coast);
      flywheel.spin(forward,8.0,voltageUnits::volt);

      enableInt = false;
    } else if(toggleF && slowF == true) {
      // if toggle on and slow
      flywheel.setStopping(coast);
      flywheel.spin(forward,6,voltageUnits::volt);
      enableInt = false;
    } else if(fly && enableFlywheelPID == false) {
      
      // flywheel.spin(forward,1.5,voltageUnits::volt);
      flywheel.setVelocity(0,percent);
      flywheel.stop();

      enableInt = true;
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
void ind() {
  int indTime = 140;

  indexer.setPosition(0,degrees);
  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec);
  indexer.spinToPosition(0,degrees);
  indexer.stop();
  
}

void fastInd() {
  fly = false;
  flywheel.setStopping(brake);
  flywheel.stop();
  flywheel.setStopping(coast);
  indexer.setStopping(hold);
  int indTime = 200;

  // flywheel.spin(forward, 9.6, voltageUnits::volt);
  flywheel.spin(forward,8, voltageUnits::volt); // 7.3

  wait(0.5, sec);

  indexer.setPosition(0,degrees);
  indexer.spin(forward, 8, voltageUnits::volt); // 8.5
  wait(indTime, msec);
  printf("1: %f\n",flywheel.voltage(voltageUnits::volt));
  printf("ind: %f\n",indexer.voltage(voltageUnits::volt));

  indexer.spin(reverse, 8, voltageUnits::volt); // 8.5
  wait(indTime, msec); 
  // indexer.stop();
  // wait(0.1, sec);
  indexer.spin(forward, 8, voltageUnits::volt);
  wait(indTime, msec);
  printf("2: %f\n",flywheel.voltage(voltageUnits::volt));
  printf("ind: %f\n",indexer.voltage(voltageUnits::volt));

  indexer.spin(reverse, 8, voltageUnits::volt);
  wait(indTime, msec);
  // indexer.stop();
  // wait(0.1, sec);
  indexer.spin(forward, 8.5, voltageUnits::volt); // 8
  wait(indTime, msec);
  printf("3: %f\n",flywheel.voltage(voltageUnits::volt));
  printf("ind: %f\n",indexer.voltage(voltageUnits::volt));

  indexer.spin(reverse, 8.5, voltageUnits::volt); // 8
  wait(indTime, msec);

  indexer.spinToPosition(0,degrees);
  
  fly = true;
  flywheel.setStopping(coast);
  flywheel.setVelocity(0,percent);
}

void farIndPID() {
  fly = false;
  // reset
  error = 0;
  prevError = 0;
  deriv = 0;
  totalError = 0;

  desiredVal = 7.5;
  kp = 0.28;
  ki = 0.9;
  kd = 0.045;

  vex::task spinfly(flyPI);
  wait(1.7, sec);

  indexer.spin(forward,4.75,voltageUnits::volt);
  wait(300,msec);
  printf("first disc\n");
  desiredVal = 7.58;
  kd = 0.01;
  kp = 0.3;
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(200,msec);
  indexer.stop();
  wait(0.8, sec);
  indexer.spin(forward,12,voltageUnits::volt);
  wait(200,msec);
  printf("second disc\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  desiredVal = 7.7;
  wait(200,msec);
  wait(0.8, sec);
  indexer.spin(forward,12,voltageUnits::volt);
  wait(200,msec);
  printf("third disc\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(200,msec);
  indexer.stop();

  vex::task::stop(spinfly); 

  indexer.spinToPosition(0,degrees);

  kp = 0;
  ki = 0;
  kd = 0;
  desiredVal = 0; 
  flywheel.stop();
}

int toggleRoll() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonL1.pressing() == true) { 
    // if buttonA is pressing:
      roller.set(true);
    } else {
    // else if buttonA is not pressing:
      roller.set(false);
    }
  
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}

int toggleInt() {  // use int for tasks (?)
  while(enableDriver) {
    if(Controller1.ButtonL2.pressing() == true && Controller1.ButtonX.pressing() == false && !lastI) { 
    // if buttonB is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = false; // set to forward
    } else if(Controller1.ButtonL2.pressing() == false && Controller1.ButtonX.pressing() == true && !lastI) {
    // else if buttonX is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = true; // set to reverse
    } else if(Controller1.ButtonL2.pressing() == false && Controller1.ButtonX.pressing() == false) {
      // else if buttons not pressing:
      lastI = false; // button was not pressed before
    }
    if(toggleI && revI == false && enableInt) {
      // if toggle on and not reverse
      intVel = 100;
      intake.setVelocity(intVel,percent);
      intake.spin(forward);
      
      // flywheel.spin(forward,2,voltageUnits::volt);
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

void auto3v2() {
  // 2 tile side
  
  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;

  fDesiredVal = 51.3;
  fkP = 0.30; //0.38
  fkI = 0.08; //0.1
  fkD = 0.03; //0.06
  
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  driveR.spinFor(reverse,2010,degrees, false); //2150 //2225
  driveL.spinFor(reverse,485,degrees, false); //700
  wait(1, sec);

  roller.set(true);
  vex::task spinfly(flywheelPID); // 1.4 sec before shoot
  wait(0.2, sec);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,150,degrees,false);
  driveR.spinFor(forward,150,degrees,false);
  wait(0.3, sec);
  
  roller.set(false);
  wait(0.2, sec);
  driveL.spinFor(forward,165,degrees);
  wait(0.3, sec);

  indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("first disc\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  indexer.stop();
  fDesiredVal = 52.8;
  wait(0.9, sec);
  indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("second disc\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  indexer.stop();

  vex::task::suspend(spinfly); 

  indexer.spinToPosition(0,degrees);

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  
  // // turn to intake
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);

  driveL.spinFor(reverse,1.135,turns,false);
  driveR.spinFor(forward,1.135,turns,false);
  wait(0.8,sec);
  // intake 
  intake.setVelocity(100,percent);
  intake.spin(forward);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,15.1,turns,false);
  driveR.spinFor(forward,15.1,turns,false);
  wait(3.1, sec); // 2.7
  // turn to shoot
  driveL.setVelocity(70,percent);
  driveR.setVelocity(70,percent);
  driveL.spinFor(forward,2.32,turns,false);
  driveR.spinFor(reverse,2.32,turns,false);
  wait(1.1, sec);
  
  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;
  // shoot 3
  fDesiredVal = 47.5;
  fkP = 0.30; //0.38
  fkI = 0.08; //0.1
  fkD = 0.04; //0.06

  // vex::task spinfly(flywheelPID);
  vex::task::resume(spinfly);
  intake.stop();
  wait(1.4, sec);

  indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("first disc\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  indexer.stop();
  fDesiredVal = 49.8;
  fkI = 0.09; //0.1
  wait(0.5, sec);
  indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("second disc\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  indexer.stop();
  fDesiredVal = 51.8;
  wait(0.5, sec);
  indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("third disc\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  wait(0.5, sec);
  indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("fourth try\n");
  indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  indexer.stop();
  vex::task::stop(spinfly); 
  indexer.spinToPosition(0,degrees);
  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
 
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
  // create tasks outside of while loop
  task toggleFlyTask(toggleFly);
  task toggleIntTask(toggleInt);
  task toggleRollTask(toggleRoll);
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
    // Controller1.ButtonR2.pressed(fwdFlywheel);
    Controller1.ButtonR1.pressed(fastIndPID);
    Controller1.ButtonUp.pressed(expansion);
    
    Controller1.ButtonDown.pressed(temp);

    wait(100, msec);
  }
}
