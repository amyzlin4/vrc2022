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
// expand               digital_out   A               
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
// bool enableInt = true;
bool toggleR = false;
bool lastR = false;
int bkwd = 1;
// bool fly = true;
bool intk = false;
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

/*
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
*/
/*
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
*/
bool enableFlywheelPID = false;

int flywheelPID(){

  while(enableFlywheelPID){
  
    double velocity = flywheel.velocity(percent);
    
    // proportional:
    fError = fDesiredVal - velocity;
    
    // derivative:
    fDerivative = fError - fPrevError;

    // integral: 
    fTotalError += fError;

    double pid = fError * fkP + fDerivative * fkD + fTotalError * fkI;
    double volt = (fDesiredVal + pid)/9;
    if (volt > 12){
      printf("volt > 10: %f\n", volt);
      volt = 12;
      
    }else if(volt < 1){
      printf("volt < 1: %f\n", volt);
      volt = 1;
    }
    flywheel.spin(forward, volt, voltageUnits::volt);
    
    printf("err: %f pid:%f cvelo: %f cvolt: %f\n",velocity, fError, pid, volt);
    // printf("%f\n", pid + desiredVal);

    fPrevError = fError;
    task::sleep(70); // delay

  }
  
  return 1;
}

void fastIndPID() {
  // fly = false;
  intk = false;

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 100;
  fkP = 200; //0.38
  fkI = 500; //0.1
  fkD = 160; //0.06
  
  enableFlywheelPID = true;
  vex::task PIDfly(flywheelPID);

  wait(1.5, sec);
  // enableInt = true;
  intVel = 70;
  // wait(1.5, sec);
  wait(0.4, sec);
  // printf("%f", intake.velocity(percent));
  // intVel = 90;
  wait(0.3, sec);
  
  // printf("%f", intake.velocity(percent));
  intVel = 100;
  wait(0.5, sec);
  printf("%f", intake.velocity(percent));

  vex::task::stop(PIDfly);
  // fly = true;
  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0;
  enableFlywheelPID = false;
  intk = true;
  // intake.stop();
  // enableInt = false;
}

void driver() {
  driveL.setStopping(brake);
  driveR.setStopping(brake);
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
    double l = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * 0.75)) / 9;
    double r = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * 0.75)) / 9;
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
/*
int toggleFly() {  // use int for tasks (?)
  while(enableDriver && !enableFlywheelPID) {
    if(Controller1.ButtonY.pressing() == true && Controller1.ButtonR2.pressing() == false && !lastF) { 
    // if buttonY is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = false; // set to normal speed
      // fly = true;
      enableFlywheelPID = false;
    } else if(Controller1.ButtonY.pressing() == false && Controller1.ButtonR2.pressing() == true && !lastF) {
    // else if buttonR2 is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = true; // set to slower speed
      // fly = true;
      enableFlywheelPID = false;

    } else if(Controller1.ButtonR2.pressing() == false && Controller1.ButtonY.pressing() == false ) {
    // else if button is not pressing:
      lastF = false; // button was not pressed before
      enableFlywheelPID = false;
      
    }

    if(toggleF && !slowF) {
      // if toggle on and not slow
      flywheel.setStopping(coast);
      flywheel.spin(forward,11.0,voltageUnits::volt);

    } else if(toggleF && slowF) {
      // if toggle on and slow
      flywheel.setStopping(coast);
      flywheel.spin(forward,10,voltageUnits::volt);
    } else  {
      // if not using PID
      flywheel.spin(forward,1.5,voltageUnits::volt);
      // flywheel.setVelocity(0,percent);
      // flywheel.stop();

    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
*/
void ind() {
  intake.setVelocity(80,percent);
  intake.spinFor(reverse,100,degrees);
  
}

/*
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
*/
/*
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
*/
/*
int toggleInt() {  // use int for tasks (?)
  while(enableDriver && !enableFlywheelPID) {
    if(Controller1.ButtonL1.pressing() == true && Controller1.ButtonX.pressing() == false && !lastI) { 
    // if buttonL1 is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before 
      revI = false; // set to intake/roller
      intk = true; 
    } else if(Controller1.ButtonL1.pressing() == false && Controller1.ButtonX.pressing() == true && !lastI) {
    // else if buttonX is pressing and was not pressed before:
      toggleI = !toggleI; // switch toggle
      lastI = true; // button was pressed before
      revI = true; // set to index
      intk = true;
    } else if(Controller1.ButtonL1.pressing() == false && Controller1.ButtonX.pressing() == false) {
      // else if buttons not pressing:
      lastI = false; // button was not pressed before
    }
    if(toggleI && !revI) {
      // if toggle on and not reverse
      intVel = 95; // intake/roller
      intake.setVelocity(intVel,percent);
      intake.spin(forward);
      
    } else if((toggleI && revI)) {
      // else if toggle on and reverse
      intVel = 85; // index
      intake.setVelocity(intVel,percent);
      intake.spin(reverse);
    } else if(intk && !enableFlywheelPID){
      intake.stop();
      
    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}
*/
void expansion() {
  if(enableDriver) {
    expand.set(true);
    wait(1,sec);
    expand.set(false);
  }
}

void temp() {
  // print to console
  printf("Right Drive: %f\n", driveR.temperature(percent));
  printf("Left Drive: %f\n", driveL.temperature(percent));
  printf("Flywheel: %f\n", flywheel.temperature(percent));
  printf("Intake: %f\n", intake.temperature(percent));
  
  // print to brain
  Brain.Screen.clearScreen();
  bool overheat = false;
  if(intake.temperature(percent) >= 50) {
    Brain.Screen.print("intake: ");
    Brain.Screen.print(intake.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(flywheel.temperature(percent) >= 50) {
    Brain.Screen.print("flywheel: ");
    Brain.Screen.print(flywheel.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(driveR.temperature(percent) >= 50) {
    Brain.Screen.print("right drive: ");
    Brain.Screen.print(driveR.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(driveL.temperature(percent) >= 50) {
    Brain.Screen.print("left drive: ");
    Brain.Screen.print(driveL.temperature(percent));
    Brain.Screen.newLine();
    overheat = true;
  }
  if(!overheat) {
    Brain.Screen.print("No overheating... yet :)");
    Brain.Screen.newLine();
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  enableDriver = false;
  flywheel.setStopping(coast);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  Inertial.calibrate();
  // expand.set(true);
}

/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/

 void printStuff() {
  //printf("%f\n", Inertial.heading(degrees));
  //  Brain.Screen.print(Inertial.heading(degrees));
  //  Brain.Screen.newLine();
  printf("L: %f   R: %f\n", driveL.power(watt), driveR.power(watt));

 }

 void turnUntil(int ang, int vel) {
   // does not work
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

void turnTo(turnType dir, double ang, int Vel) {
  // in progress
  if(dir == right)
  {
    while(Inertial.heading(degrees) < ang)
    {
      driveL.spin(forward, Vel, percent);
      driveR.spin(reverse, Vel, percent);
    }
  } 
  else if (dir == left)
  {
    while(Inertial.heading(degrees) > ang)
    {
      driveL.spin(reverse, Vel, percent);
      driveR.spin(forward, Vel, percent);
    }
  }
  
  driveL.stop();
  driveR.stop();
  printf("%f\n", Inertial.heading(degrees));
}

void turnFor(double ang, int vel) {
  // works ok
  driveL.setVelocity(vel,percent);
  driveR.setVelocity(vel,percent);
  driveL.spinFor(forward,ang * TURN,degrees, false);
  driveR.spinFor(reverse,ang * TURN,degrees, false);
  printf("%f\n", Inertial.heading(degrees));
}
/*
void pDrive() {
  // works well in auton
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
*/
/*
void goTo(double x, double y) {
  // in progress 
  double bx = x - GPS.xPosition(inches);
  double cy = y - GPS.yPosition(inches);
  // calculate 
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
*/
void curveRight(double angl, double t, double r) {
  /*
  angl in radians
  t (time) in sec
  r (radius) in inches
  */
  double vL = (trackWidth + r) * angl / t;
  double vR = r * angl / t;
  driveL.setVelocity(vL, percent);
  driveR.setVelocity(vR, percent);
  driveL.spin(forward);
  driveR.spin(forward);
  wait(t, sec);
  driveL.stop();
  driveR.stop();
}

void curveLeft(double angl, double t, double r) {
  double vL = r * angl / t;
  double vR = (trackWidth + r) * angl / t;
  driveL.setVelocity(vL, percent);
  driveR.setVelocity(vR, percent);
  driveL.spin(forward);
  driveR.spin(forward);
  wait(t, sec);
  driveL.stop();
  driveR.stop();
}

/*================================================================================================*/
// ENCODER STUFF:
double toInch = 1.0;

int printM() {
  while(true)
  {
    //printf("R: %.3f  L: %.3f\n", driveR.voltage(voltageUnits::volt), driveL.voltage(voltageUnits::volt));
    printf("R: %.3f   L: %.3f\n", EncoderR.position(degrees), EncoderL.position(degrees));
  }
  return 1;
}

int drivePID(){

  while(enableDrivePID){
   
    vex::task printMtask(printM);
    double valR = EncoderR.position(degrees);
    double valL = EncoderL.position(degrees);
    double avgVal = (valR + valL) / 2;

    // proportional:
    dErr = dDesiredVal - avgVal;
    
    // derivative:
    dDeriv = dErr - dPrevErr;

    // integral: 
    dTotalErr += dErr;

    double pid = dErr * dkP + dDeriv * dkD + dTotalErr * dkI;
    //printf("E: %f, D: %f, TE: %f, pid: %f\n", fError, fDeriv, fTotalError, pid);
    double volt = (pid)/10;
    
    if (volt > 8){
      printf("volt > 10: %f\n", volt);
      volt = 8;
      
    }
    
    else if(volt < 0.5){
      printf("volt < 0.5: %f\n", volt);
      volt = 0.5;
    }
    
    //flywheel.spin(forward, volt, voltageUnits::volt);
    driveL.spin(forward, volt * 1.005, voltageUnits::volt);
    driveR.spin(forward, volt, voltageUnits::volt);

    
    //printf("err: %f pid:%f cvelo: %f cvolt: %f %f\n",velocity, fError, pid, volt, totalTimesShot);
    printf("Set: %f   L: %.3f   R: %.3f\n", volt, driveL.voltage(voltageUnits::volt), driveR.voltage(voltageUnits::volt));

    dPrevErr = dErr;
    task::sleep(50); // delay
    vex::task::suspend(printMtask);


  }
  
  return 1;
}

void printEncoder() {
  double r = EncoderR.position(degrees);
  double l = EncoderL.position(degrees);
  printf("R: %.3f  L: %.3f\n", r, l);
  if(r > l)
  {
    printf("%.3f left\n", r-l);
  }
  else if(l > r)
  {
    printf("%.3f right\n", l-r);
  }
}

void test1() {
  printEncoder();

  // reset
  dErr = 0;
  dPrevErr = 0;
  dDeriv = 0;
  dTotalErr = 0;

  dkP = 3; //
  dkI = 1; //
  dkD = 45; //

  dDesiredVal = 1500;
  enableDrivePID = true;
  vex::task drivePIDtask(drivePID);
  wait(2, sec);
  vex::task::suspend(drivePIDtask);
  dkP = 0.0; //
  dkI = 0.0; //
  dkD = 0.0; //
  driveL.stop();
  driveR.stop();


}

void test2() {
  
  vex::task printMtask(printM);
  driveL.spin(forward, 7, voltageUnits::volt);
  driveR.spin(forward, 7, voltageUnits::volt);
  // driveL.spin(forward, 70, percent);
  // driveR.spin(forward, 70, percent);
  wait(2, sec);
  // wait(1, sec);
  // driveL.spin(forward, 8, voltageUnits::volt);
  // driveR.spin(forward, 8, voltageUnits::volt);
  // wait(0.15, sec);
  // driveL.spin(forward, 7, voltageUnits::volt);
  // driveR.spin(forward, 7, voltageUnits::volt);
  // wait(0.15, sec);
  // driveL.spin(forward, 6, voltageUnits::volt);
  // driveR.spin(forward, 6, voltageUnits::volt);
  // wait(0.15, sec);
  // driveL.spin(forward, 5, voltageUnits::volt);
  // driveR.spin(forward, 5, voltageUnits::volt);
  // wait(0.15, sec);
  // driveL.spin(forward, 4, voltageUnits::volt);
  // driveR.spin(forward, 4, voltageUnits::volt);
  // wait(0.15, sec);
  // driveL.spin(forward, 3, voltageUnits::volt);
  // driveR.spin(forward, 3, voltageUnits::volt);
  // wait(0.3, sec);
  driveL.stop();
  driveR.stop();
  vex::task::suspend(printMtask);
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

  fDesiredVal = 52.6;
  fkP = 0.30; //0.38
  fkI = 0.08; //0.1
  fkD = 0.03; //0.06

  // set indexer if necessary
   
  // roller.set(true);
  wait(0.2, sec);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,300,degrees,false);
  driveR.spinFor(forward,300,degrees,false);
  wait(0.5, sec);
  vex::task spinfly(flywheelPID);
  // roller.set(false);
  wait(0.2, sec);

  wait(1, sec);

  // index one
  wait(95,msec);
  printf("first disc\n");
  // index back
  wait(140,msec);
  // index stop
  fDesiredVal = 51.7;
  wait(0.9, sec);
  // index one
  wait(140,msec);
  printf("second disc\n");
  // index back
  wait(140,msec);
  // index stop

  vex::task::suspend(spinfly); 

  // indexer reset if necessary

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
}

void auto2v2() {
  // 3 tile side 
  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;

  fDesiredVal = 52.6;
  fkP = 0.30; //0.38
  fkI = 0.08; //0.1
  fkD = 0.03; //0.06

  
   
  // roller.set(true);
  wait(0.2, sec);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,300,degrees,false);
  driveR.spinFor(forward,300,degrees,false);
  wait(0.5, sec);
  vex::task spinfly(flywheelPID);
  // roller.set(false);
  wait(0.2, sec);

  wait(1, sec);

  // indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("first disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  fDesiredVal = 51.7;
  wait(0.9, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("second disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();

  vex::task::suspend(spinfly); 

  // indexer.spinToPosition(0,degrees);

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  
  // turn to intake
  
  driveL.setVelocity(63,percent);
  driveR.setVelocity(63,percent);
  driveL.spinFor(forward,1.257,turns,false);
  driveR.spinFor(reverse,1.257,turns,false);
  wait(0.7, sec);
  // intake 3
  intake.spin(forward,12,voltageUnits::volt);
  driveL.spin(forward,50,percent);
  driveR.spin(forward,50,percent);
  wait(1.09, sec); // 1.1 at 50
  driveL.spin(forward,15,percent);
  driveR.spin(forward,15,percent);
  wait(1.52,sec); // 1.5 at 15
  driveL.spin(forward,50,percent);
  driveR.spin(forward,50,percent);
  wait(0.4, sec);
  // turn to shoot
  driveL.stop();
  driveR.stop();
  wait(0.5,sec);
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  
  driveL.spinFor(reverse,1.91,turns,false);
  driveR.spinFor(forward,1.91,turns,false);
  wait(0.9, sec);
  intake.stop();
  // shoot 3
  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;
  // shoot 3
  fDesiredVal = 47;
  fkP = 0.30; //0.38
  fkI = 0.08; //0.1
  fkD = 0.04; //0.06

  // vex::task spinfly(flyPI);
  vex::task::resume(spinfly);
  intake.stop();
  wait(1.5, sec);

  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("first disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  fDesiredVal = 50;
  wait(0.6, sec);
  // indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("second disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  fDesiredVal = 51;
  wait(0.6, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("third disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  wait(0.6, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("fourth try\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  vex::task::stop(spinfly); 
  // indexer.spinToPosition(0,degrees);
  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  
}

void auto3v1() {
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
  driveR.spinFor(reverse,2000,degrees, false); //2150 //2225
  driveL.spinFor(reverse,475,degrees, false); //700
  wait(1, sec);

  // roller.set(true);
  vex::task spinfly(flywheelPID); // 1.4 sec before shoot
  wait(0.2, sec);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,150,degrees,false);
  driveR.spinFor(forward,150,degrees,false);
  wait(0.3, sec);
  
  // roller.set(false);
  wait(0.2, sec);
  driveL.spinFor(forward,175,degrees);
  wait(0.3, sec);

  // indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("first disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  fDesiredVal = 52.8;
  wait(0.9, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("second disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();

  vex::task::suspend(spinfly); 

  // indexer.spinToPosition(0,degrees);

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  
}

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
  driveR.spinFor(reverse,2000,degrees, false); //2150 //2225
  driveL.spinFor(reverse,475,degrees, false); //700
  wait(1, sec);

  // roller.set(true);
  vex::task spinfly(flywheelPID); // 1.4 sec before shoot
  wait(0.2, sec);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,150,degrees,false);
  driveR.spinFor(forward,150,degrees,false);
  wait(0.3, sec);
  
  // roller.set(false);
  wait(0.2, sec);
  driveL.spinFor(forward,175,degrees);
  wait(0.3, sec);

  // indexer.spin(forward,11,voltageUnits::volt);
  wait(95,msec);
  printf("first disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  fDesiredVal = 52.8;
  wait(0.9, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("second disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();

  vex::task::suspend(spinfly); 

  // indexer.spinToPosition(0,degrees);

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

  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("first disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  fDesiredVal = 49.8;
  fkI = 0.09; //0.1
  wait(0.5, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("second disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  fDesiredVal = 51.8;
  wait(0.5, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("third disc\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  wait(0.5, sec);
  // indexer.spin(forward,8,voltageUnits::volt);
  wait(140,msec);
  printf("fourth try\n");
  // indexer.spin(reverse,8,voltageUnits::volt);
  wait(140,msec);
  // indexer.stop();
  vex::task::stop(spinfly); 
  // indexer.spinToPosition(0,degrees);
  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
 
}

/*
void auto3v3() {
  // 2 tile side
  // reset
  error = 0;
  prevError = 0;
  deriv = 0;
  totalError = 0;

  desiredVal = 7.5;
  kp = 0.28;
  ki = 0.9;
  kd = 0.045;

  driveL.setStopping(coast);
  driveR.setStopping(coast);
  driveL.setVelocity(65,percent);
  driveR.setVelocity(65,percent);
  driveR.spinFor(reverse,2150,degrees, false);
  driveL.spinFor(reverse,700,degrees, false);
  wait(1.2, sec);

  roller.set(true);
  vex::task spinfly(flyPI);
  wait(0.2, sec);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,150,degrees,false);
  driveR.spinFor(forward,150,degrees,false);
  wait(0.3, sec);
  
  roller.set(false);
  wait(0.2, sec);
  driveL.spinFor(forward,250,degrees);
  wait(0.9, sec);

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
  wait(200,msec);
  indexer.stop();

  vex::task::suspend(spinfly); 

  indexer.spinToPosition(0,degrees);

  kp = 0;
  ki = 0;
  kd = 0;
  desiredVal = 0; 
  flywheel.stop();
  
  // // turn to intake
  driveL.setVelocity(60,percent);
  driveR.setVelocity(60,percent);
  // driveL.spinFor(reverse,1.22,turns,false);
  // driveR.spinFor(forward,1.22,turns,false);
  // wait(1, sec);
  driveL.spinFor(reverse,1.31,turns,false);
  driveR.spinFor(forward,1.31,turns,false);
  wait(1.1,sec);
  // intake 
  flywheel.spin(reverse,3,voltageUnits::volt);
  intake.setVelocity(100,percent);
  intake.spin(forward);
  driveL.setVelocity(70,percent);
  driveR.setVelocity(70,percent);
  driveL.spinFor(forward,17,turns,false);
  driveR.spinFor(forward,17,turns,false);
  wait(2.4, sec);
  intake.stop();
  flywheel.stop();
  // 180
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(reverse,4.15,turns,false);
  driveR.spinFor(forward,4.15,turns,false);
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
*/

void autonomous(void) {
  enableDriver = false;
  //auto2v2();
  // test2();

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
  
  intVel = 90;
  // create tasks outside of while loop
  // task toggleFlyTask(toggleFly);
  // task toggleIntTask(toggleInt);
  // task::setPriority(toggleFlyTask,2);
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
    Controller1.ButtonR1.pressed(fastIndPID);
    Controller1.ButtonUp.pressed(expansion);
    
    Controller1.ButtonDown.pressed(temp);
    Controller1.ButtonRight.pressed(test2);
    Controller1.ButtonLeft.pressed(printEncoder);

    wait(100, msec);
  }
}
