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
// InertialL            inertial      1               
// MotorR1              motor         8               
// MotorR2              motor         9               
// MotorR3              motor         10              
// flywheel             motor         2               
// angler               digital_out   A               
// expand               digital_out   B               
// MotorL1              motor         11              
// MotorL2              motor         12              
// MotorL3              motor         13              
// InertialR            inertial      20              
// Controller2          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

bool toggleF = false;
bool lastF = false;
bool slowF = false;
bool fly = true;
// bool enableFlyPID = false;

bool toggleI = false;
bool lastI = false;
bool revI = false;
int intVel = 100;
bool enableInt = true;

double DRIVE = 0.014; // update
double TURN = 8; // update
bool enableDriver = true;

bool toggleA = false;
bool lastA = false;

bool lastE = false;
bool toggleE = false;
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
double dLkP = 0.01;
double dLkI = 0.0;
double dLkD = 0.03;
double dLDeriv;
double dLDesiredVal;
double dLErr;
double dLPrevErr = 0;
double dLTotalErr = 0;

double dRkP = 0.01;
double dRkI = 0.0;
double dRkD = 0.03;
double dRDeriv;
double dRDesiredVal;
double dRErr;
double dRPrevErr = 0;
double dRTotalErr = 0;

double turnKP = 0.107; // 0.2
double turnKI = 0.0;
double turnKD = 0.33; // 0.05
double turnError;      // sensorValue - DesiredValue: a position value
double turnPrevError = 0;  // position 20 milliseconds ago
double turnDerivative; // error - prevError: speed
double turnTotalError = 0; // totalError + error

/*===================================================================*/
// DRIVETRAIN:
// double wheelTravel = 12.56; // circumference 
double trackWidth = 1.0; // front edge, wheel to wheel
// double trackLength = 12; // side edge, axle to axle
motor_group driveL = motor_group(MotorL1, MotorL2, MotorL3);
motor_group driveR = motor_group(MotorR1, MotorR2, MotorR3);

// DRIVER CONTROL FUNCTIONS:
/*================================================================================================*/

bool enableDriveLPID = true;
bool enableDriveRPID = true;
bool enableTurnLPID = false;
bool enableTurnRPID = false;
bool enableTurnPID = false;

double desiredValue = 200;
double desiredTurnValue = 0; //set it to whatever absolute angle (in relation to the orientation while initialzing) you want. The actual turn position is then your inertial sensor's rotation value. 

bool resetDriveLSensors = false;
bool resetDriveRSensors = false;
bool resetDriveSensors = false;
///////////////////////////////////////////////////////////////////
int driveLPID() {

  while (enableDriveLPID) {

    if (resetDriveLSensors) {
      resetDriveLSensors = false;

      driveL.setPosition(0, degrees);
      InertialL.setHeading(0,degrees);

      dLDeriv = 0;
      dLErr = 0;
      dLPrevErr = 0;
      dLTotalErr = 0;

    }

    //current rotational position of the first motor in a motor group:
    double leftMotorPosition = -driveL.position(degrees);
    printf("driveL: %.3f\n", leftMotorPosition);

    /////////////////////////////////
    //  lateral movement PID
    ////////////////////////////////

    dLErr = leftMotorPosition - dLDesiredVal; // Proportional (P)
    dLDeriv = dLErr - dLPrevErr; // derivative (D)
    dLTotalErr += dLErr;  // integral (I)

    double lateralMotorPower = dLErr * dLkP + dLDeriv * dLkD + dLTotalErr * dLkI;
    // if to do degrees than voltage, just /360 instead of /12

    /////////////////////////////////
    //  pointing PID
    ////////////////////////////////

    double turnDifference = InertialL.heading(degrees);
    if(turnDifference > 180)
    {
      turnDifference = 360 - turnDifference;
    }
    printf("Inertial: %.3f\n", turnDifference);

    turnError = turnDifference - desiredTurnValue; // (P)
    turnDerivative = turnError - turnPrevError; // derivative (D)
    turnTotalError += turnError;  // integral (I) ----> considering not using it for drivetrain, PD control

    double turnMotorPower = turnError * turnKP + turnDerivative * turnKD + turnTotalError * turnKI;
    turnMotorPower = 0;


    /////////////////////////////////
    //   overall
    ////////////////////////////////
    driveL.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    double outHeading = InertialL.heading(degrees);
    printf("L output: %.3f\nHeading: %.3f\n", lateralMotorPower + turnMotorPower,  outHeading);

    dLPrevErr = dLErr;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}

int driveRPID() {

  while (enableDriveRPID) {

    if (resetDriveRSensors) {
      resetDriveRSensors = false;

      driveR.setPosition(0, degrees);
      InertialR.setHeading(0,degrees);

      dRDeriv = 0;
      dRErr = 0;
      dRPrevErr = 0;
      dRTotalErr = 0;

    }

    //current rotational position of the first motor in a motor group:
    double rightMotorPosition = -driveR.position(degrees);
    printf("driveR: %.3f\n",  rightMotorPosition);

    /////////////////////////////////
    //  lateral movement PID
    ////////////////////////////////

    dRErr = rightMotorPosition - dRDesiredVal; // Proportional (P)
    dRDeriv = dRErr - dRPrevErr; // derivative (D)
    dRTotalErr += dRErr;  // integral (I)

    double lateralMotorPower = dRErr * dRkP + dRDeriv * dRkD + dRTotalErr * dRkI;
    // if to do degrees than voltage, just /360 instead of /12

    /////////////////////////////////
    //  pointing PID
    ////////////////////////////////

    double turnDifference = InertialR.heading(degrees);
    if(turnDifference > 180)
    {
      turnDifference = 360 - turnDifference;
    }
    printf("Inertial: %.3f\n", turnDifference);

    turnError = turnDifference - desiredTurnValue; // (P)
    turnDerivative = turnError - turnPrevError; // derivative (D)
    turnTotalError += turnError;  // integral (I) ----> considering not using it for drivetrain, PD control

    double turnMotorPower = turnError * turnKP + turnDerivative * turnKD + turnTotalError * turnKI;
    turnMotorPower = 0;


    /////////////////////////////////
    //   overall
    ////////////////////////////////
    driveR.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    double outHeading = InertialR.heading(degrees);
    printf("R output: %.3f\nHeading: %.3f\n", lateralMotorPower - turnMotorPower, outHeading);

    dRPrevErr = dRErr;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}


int turnLeftPID(){

  while(enableTurnLPID){
    
    if (resetDriveSensors){
      resetDriveSensors = false;

      driveR.setPosition(0, degrees);
      driveL.setPosition(0, degrees);
      // InertialL.setHeading(0,degrees);

      turnError = 0; 
      turnPrevError = 0;  
      turnDerivative = 0; 
      turnTotalError = 0;

    }

    double inertialHeading = InertialL.heading(degrees);

    if(inertialHeading < 180){
      inertialHeading = 360 - inertialHeading;
    }

    //positional
    turnError = inertialHeading - desiredTurnValue;
    
    //derivative
    turnDerivative = turnError - turnPrevError;

    //integral
    turnTotalError = turnTotalError + turnError;

    
    double turnTotalMotorPosition = turnError * turnKP + turnDerivative * turnKD + turnTotalError * turnKI;
    printf("inertialHeading: %f desired val: %f turn: %f error: %f\n", InertialL.heading(degrees), desiredTurnValue, turnTotalMotorPosition, turnError);
  
    driveR.spin(forward, 0 + turnTotalMotorPosition, voltageUnits::volt);
    driveL.spin(forward, 0 - turnTotalMotorPosition, voltageUnits::volt);


    turnPrevError = turnError;
    vex::task::sleep(20);
  
  }
  return 1;
}

int turnRightPID(){

  while(enableTurnRPID){
    
    if (resetDriveSensors){
      resetDriveSensors = false;

      driveR.setPosition(0, degrees);
      driveL.setPosition(0, degrees);
      // InertialL.setHeading(0,degrees);

      turnError = 0; 
      turnPrevError = 0;  
      turnDerivative = 0; 
      turnTotalError = 0;

    }


    //positional
    turnError = InertialL.heading(degrees) - desiredTurnValue;
    
    //derivative
    turnDerivative = turnError - turnPrevError;

    //integral
    turnTotalError = turnTotalError + turnError;

    
    double turnTotalMotorPosition = turnError * turnKP + turnDerivative * turnKD + turnTotalError * turnKI;
    printf("inertialHeading: %f desired val: %f turn: %f error: %f\n", InertialL.heading(degrees), desiredTurnValue, turnTotalMotorPosition, turnError);
  
    driveR.spin(forward, 0 + turnTotalMotorPosition, voltageUnits::volt);
    driveL.spin(forward, 0 - turnTotalMotorPosition, voltageUnits::volt);


    turnPrevError = turnError;
    vex::task::sleep(20);
  
  }
  return 1;
}


int turnPID(){

  while(enableTurnPID){
    
    if (resetDriveSensors){
      resetDriveSensors = false;

      driveR.setPosition(0, degrees);
      driveL.setPosition(0, degrees);
      // InertialL.setHeading(0,degrees);

      turnError = 0; 
      turnPrevError = 0;  
      turnDerivative = 0; 
      turnTotalError = 0;

    }

    double headingL = InertialL.heading(degrees);
    double headingR = InertialR.heading(degrees);
    
    if(headingL > 180)
    {
      headingL = headingL - 360;
    }
    if(headingR > 180)
    {
      headingR = headingR - 360;
    }

    // if(headingL <= 1 && ((headingR-headingL) > 5 || (headingL-headingR) > 5))
    // {
    //   headingL = headingR;
    // }
    // else if(headingR <= 1 && ((headingR-headingL) > 5 || (headingL-headingR) > 5))
    // {
    //   headingR = headingL;
    // }

    // if(headingL < 5 || headingR < 5)
    // {
    //   enableTurnPID = false;
    // }

    double head = (headingL + headingR)/2;
    printf("Head:%f   L:%f   R:%f\n", head, headingL, headingR);

    //positional
    turnError = head - desiredTurnValue;
    
    //derivative
    turnDerivative = turnError - turnPrevError;

    //integral
    turnTotalError = turnTotalError + turnError;

    
    double turnTotalMotorPosition = turnError * turnKP + turnDerivative * turnKD + turnTotalError * turnKI;
    // printf("inertialHeading: %f desired val: %f turn: %f error: %f\n", head, desiredTurnValue, turnTotalMotorPosition, turnError);
  
    driveR.spin(forward, 0 + turnTotalMotorPosition, voltageUnits::volt);
    driveL.spin(forward, 0 - turnTotalMotorPosition, voltageUnits::volt);


    turnPrevError = turnError;
    vex::task::sleep(20);
  
  }
  return 1;
}


/////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////
void driver() {
  driveL.setStopping(coast);
  driveR.setStopping(coast);
  if((Controller1.Axis3.position(percent) != 0))
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
  else if((Controller1.Axis1.position(percent) != 0))
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

void tank() {
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
    if(Controller1.ButtonY.pressing()&& !lastF) { 
    // if buttonY is pressing and button was not pressed before:
      toggleF = !toggleF; // switch toggle
      lastF = true; // button was pressed before
      slowF = true; // set to normal speed
      fly = true;
      // enableFlyPID = false;
      enableFlywheelPID = false;
      Controller1.rumble(rumbleShort);

    } else if(!Controller1.ButtonY.pressing()) {
    // else if button is not pressing:
      lastF = false; // button was not pressed before
      ///////// try this:
      // enableFlyPID = false;
      enableFlywheelPID = false; // remove?
      // fly = true;
      // flywheel.spin(forward,1.5,voltageUnits::volt);
    }
    
    if(toggleF && slowF) {
      // if toggle on and not slow
      flywheel.setStopping(coast);
      flywheel.spin(forward,8.0,voltageUnits::volt);

    
    } else if(fly && !enableFlywheelPID) {
      
      flywheel.spin(forward,6.0,voltageUnits::volt);
      // flywheel.setVelocity(0,percent);
      // flywheel.stop();

    }
    task::sleep(50); // delay
  }
  return 1; // all tasks must return (?)
}

void ind() {
  enableInt = true;
  intake.spin(reverse,8,voltageUnits::volt);
  wait(0.14, sec);
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

////////////////////////////////////////////////////////////
void tripShot1() {
  // normal/short range
  fly = false;

  driveL.setStopping(hold);
  driveR.setStopping(hold);

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  toggleA = false;
  angler.set(false); // angle up
  
  wait(0.17, sec); //0.5
  enableInt = true;
  intake.spin(reverse,10,voltageUnits::volt); //10
 
  wait(1.2, sec);
  
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

void tripShot2() {
  // mid/long range
  fly = false;

  driveL.setStopping(hold);
  driveR.setStopping(hold);

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  toggleA = false;
  angler.set(false); // angle up
  
  wait(0.4, sec); //0.5
  enableInt = true;
  intake.spin(reverse,7,voltageUnits::volt); //10
 
  //wait(1.4, sec);

  wait(0.15, sec);
  enableInt = false;
  wait(0.1, sec);
  enableInt = true;
  wait(0.15, sec);
  enableInt = false;
  wait(0.1, sec);
  enableInt = true;
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

void tripShot3() {
  // overfill (shortest)
  fly = false;

  driveL.setStopping(hold);
  driveR.setStopping(hold);

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  toggleA = false;
  angler.set(false); // angle up
  
  wait(0.2, sec); //0.5
  enableInt = true;
  intake.spin(reverse,8,voltageUnits::volt); //10
 
  wait(1.2, sec);
  
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
///////////////////////////////////////////////////////////
void tripShotA() {
  // normal/short range
  fly = false;

  driveL.setStopping(hold);
  driveR.setStopping(hold);

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  toggleA = false;
  angler.set(false); // angle up
  
  wait(0.3, sec); //0.5
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt); //10
 
  wait(1.2, sec);
  
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

void tripShotB() {
  // mid/long range
  fly = false;

  driveL.setStopping(hold);
  driveR.setStopping(hold);

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  toggleA = true;
  angler.set(true); // angle down
  
  wait(0.3, sec); //0.5
  enableInt = true;
  intake.spin(reverse,9,voltageUnits::volt); //10
 
  //wait(1.4, sec);

  wait(0.15, sec);
  enableInt = false;
  wait(0.1, sec);
  enableInt = true;
  wait(0.15, sec);
  enableInt = false;
  wait(0.1, sec);
  enableInt = true;
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

void tripShotC() {
  // overfill (shortest)
  fly = false;

  driveL.setStopping(hold);
  driveR.setStopping(hold);

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  toggleA = false;
  angler.set(false); // angle up
  
  wait(0.05, sec); //0.1
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt); //10
 
  wait(1, sec);
  
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
//////////////////////////////////////////////////////////

void farShot() {
  fly = false;

  // reset
  fError = 0;
  fPrevError = 0;
  fDerivative = 0;
  fTotalError = 0;

  fDesiredVal = 80;//80
  fkP = 0.48; //0.38
  fkI = 0; //0.1
  fkD = 0.1; //0.06
  
  enableFlywheelPID = true;
  // vex::task PIDfly(flywheelPID);
  // vex::task PIDfly(flyPI);
  flywheel.spin(forward,12,voltageUnits::volt);
  
  wait(1, sec); 
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt);
 
  wait(0.15, sec);
  
  enableInt = false;
  wait(0.5, sec);
  enableInt = true;
  intake.spin(reverse,12,voltageUnits::volt);
  wait(0.16, sec);

  enableInt = false;
  wait(0.5, sec);
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
    Brain.Screen.print("expand");
    Brain.Screen.newLine();
    expand.set(true);
    wait(1,sec);
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
  Controller1.Screen.clearLine();
  Controller1.Screen.print("FW: %f", flywheel.temperature(percent));

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
  InertialL.calibrate();
  InertialR.calibrate();
  angler.set(false);
  expand.set(false);
}

/* --------------------------------------------------------------------------*/
// Autonomous
/* --------------------------------------------------------------------------*/



/*================================================================================================*/
/* TOURNAMENT AUTONS                                                                              */
/*================================================================================================*/

void l9v1() {
  
  // starting position: facing disc

  angler.set(true);

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

  driveL.setVelocity(30,percent);
  driveR.setVelocity(30,percent);
  driveL.setStopping(coast);
  driveR.setStopping(coast);

  intake.spin(forward,12,voltageUnits::volt);
  driveL.spinFor(forward, 200,degrees,false);
  driveR.spinFor(forward, 200,degrees,false);
  wait(0.5, sec);

  driveL.spinFor(reverse, 100,degrees,false);
  driveR.spinFor(reverse, 100,degrees,false);
  wait(0.3, sec);

  // turn to roller
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 165,degrees,false);
  driveR.spinFor(reverse, 165,degrees,false);
  wait(0.4, sec);

  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;

  turnKP = 0.15;

  desiredTurnValue = 32;
  wait(0.2, sec);

  vex::task::suspend(turnLTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  turnKP = 0.107;

  // roll
  driveL.spinFor(reverse, 200,degrees,false);
  driveR.spinFor(reverse, 200,degrees,false);
  wait(0.5, sec);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 150,degrees,false);
  wait(0.3, sec);
  intake.stop();

  // shoot
  // intake.spin(forward, 100, percent);
  wait(0.2, sec);
  // intake.stop();
  // wait(0.1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(forward, 100, percent);
  wait(0.12, sec);
  intake.stop();
  wait(0.25, sec);

  intake.spin(reverse,100,percent);
  wait(0.13, sec);
  intake.stop(); 
  wait(0.1, sec);
 
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.4, sec);

  intake.spin(reverse,80,percent);
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

  // intake
  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -2000; // -2000
  dRDesiredVal = 590; // 590
  wait(0.15, sec); // 0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask(turnPID);
  desiredTurnValue = 75; //70?
  wait(0.3, sec); //0.2?
  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  dRkP = 0.004; //0.004
  dLkP = 0.004;
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -2800; //-2800
  dRDesiredVal = -2800; //-2450
  desiredTurnValue = 0; 
  wait(1, sec); //1

  dRkP = 0.0075;
  dLkP = 0.0075;
  wait(0.31, sec); //0.5

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

  vex::task::resume(spinfly);
  wait(0.2, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 1.1,turns,false);
  driveR.spinFor(forward, 1.1,turns,false);
  wait(0.37, sec);
  driveL.stop();
  driveR.stop();
  
  enableTurnPID = true;
  vex::task turnLTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -93;

  wait(0.27, sec);

  vex::task::stop(turnLTask1);
  enableTurnLPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.2, sec);
  

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.05, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.15, sec);

  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop(); 
  wait(0.05, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.15, sec);

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
  wait(0.05, sec);  


  driveR.spinFor(reverse,230,degrees);
  wait(0.2, sec);

  // intake
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);

  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.008;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 2300; 
  dRDesiredVal = 2900; 
  wait(0.8, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  wait(0.4, sec);

  
  driveL.setVelocity(100,percent);
  driveL.spinFor(forward,0.1,turns, false);
  driveR.setVelocity(100,percent);
  driveR.spinFor(reverse,0.1,turns, false);
  wait(0.1, sec);
  driveR.stop();
  driveL.stop();

  enableTurnPID = true;
  vex::task turnRTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 13;

  wait(0.2, sec);

  vex::task::stop(turnRTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask2(driveLPID);
  vex::task driveRTask2(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  intake.spin(forward,12,voltageUnits::volt);
  dRkP = 0.002;
  dLkP = 0.002;
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3200; 
  dRDesiredVal = -2800;
  desiredTurnValue = 0; 
  wait(0.7, sec);

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

  vex::task::resume(spinfly);
  // vex::task spinfly(flyPI);

  // dLDesiredVal = -2800;
  dRkP = 0.003;
  dLkP = 0.003;
  wait(0.8, sec);

  vex::task::stop(driveLTask2); 
  vex::task::stop(driveRTask2);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.setStopping(hold);
  driveR.stop();
  wait(0.5, sec);

  // turn to shoot
  // InertialL.setHeading(0,degrees);
  // InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveL.spinFor(reverse, 1.0,turns,false);
  // driveR.setVelocity(90,percent);
  // driveR.spinFor(forward, 0.2,turns,false);
  wait(0.4, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnLTask2(turnPID);
  resetDriveSensors = true;
  turnKP = 0.13;
  desiredTurnValue = -50;

  wait(0.5, sec);

  vex::task::stop(turnLTask2);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.05, sec);
  intake.stop();
  wait(0.1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.15, sec);

  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.1, sec);

  intake.spin(reverse,100,percent);
  wait(0.2, sec);
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


void l6v1() {
  
  // starting position: facing disc

  angler.set(true);

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

  driveL.setVelocity(30,percent);
  driveR.setVelocity(30,percent);
  driveL.setStopping(coast);
  driveR.setStopping(coast);

  intake.spin(forward,12,voltageUnits::volt);
  driveL.spinFor(forward, 200,degrees,false);
  driveR.spinFor(forward, 200,degrees,false);
  wait(0.5, sec);

  driveL.spinFor(reverse, 100,degrees,false);
  driveR.spinFor(reverse, 100,degrees,false);
  wait(0.3, sec);

  // turn to roller
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 165,degrees,false);
  driveR.spinFor(reverse, 165,degrees,false);
  wait(0.4, sec);

  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;

  turnKP = 0.15;

  desiredTurnValue = 32;
  wait(0.2, sec);

  vex::task::suspend(turnLTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  turnKP = 0.107;

  // roll
  driveL.spinFor(reverse, 200,degrees,false);
  driveR.spinFor(reverse, 200,degrees,false);
  wait(0.5, sec);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 150,degrees,false);
  wait(0.3, sec);
  intake.stop();

  // shoot
  // intake.spin(forward, 100, percent);
  wait(0.2, sec);
  // intake.stop();
  // wait(0.1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(forward, 100, percent);
  wait(0.12, sec);
  intake.stop();
  wait(0.35, sec);

  intake.spin(reverse,100,percent);
  wait(0.13, sec);
  intake.stop(); 
  wait(0.1, sec);
 
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.45, sec);

  intake.spin(reverse,80,percent);
  wait(0.4, sec);
  intake.stop(); 
  wait(0.1, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;

  // intake
  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  // intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -2000; // -2000
  dRDesiredVal = 590; // 590
  wait(0.15, sec); // 0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask(turnPID);
  desiredTurnValue = 77; //70?
  wait(0.3, sec); //0.2?
  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  dRkP = 0.006; //0.004
  dLkP = 0.006;
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3200; //-2800
  dRDesiredVal = -3200; //-2450
  desiredTurnValue = 0; 
  wait(0.7, sec); //1
  intake.spin(forward,12,voltageUnits::volt);


  dRkP = 0.004;
  dLkP = 0.004;
  wait(1.3, sec); //0.5

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

  vex::task::resume(spinfly);
  wait(0.2, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 1.1,turns,false);
  driveR.spinFor(forward, 1.1,turns,false);
  wait(0.37, sec);
  driveL.stop();
  driveR.stop();
  
  enableTurnPID = true;
  vex::task turnLTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -93;

  wait(0.27, sec);

  vex::task::stop(turnLTask1);
  enableTurnLPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.2, sec);
  

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.05, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.25, sec);

  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop(); 
  wait(0.05, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.25, sec);

  intake.spin(reverse,100,percent);
  wait(0.4, sec);
  intake.stop(); 
  wait(0.1, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  wait(0.05, sec);  



}


void l9v2() {
  
  // starting position: facing disc

  angler.set(true);

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

  driveL.setVelocity(30,percent);
  driveR.setVelocity(30,percent);
  driveL.setStopping(coast);
  driveR.setStopping(coast);

  intake.spin(forward,12,voltageUnits::volt);
  driveL.spinFor(forward, 200,degrees,false);
  driveR.spinFor(forward, 200,degrees,false);
  wait(0.5, sec);

  driveL.spinFor(reverse, 100,degrees,false);
  driveR.spinFor(reverse, 100,degrees,false);
  wait(0.3, sec);

  // turn to roller
  driveL.setVelocity(70,percent);
  driveR.setVelocity(70,percent);
  driveL.spinFor(forward, 170,degrees,false);
  driveR.spinFor(reverse, 170,degrees,false);
  wait(0.5, sec);
  // turnKP = 0.095;
  // turnKI = 0.000;
  // turnKD = 0.005;

  // enableTurnLPID = true;
  // vex::task turnLTask(turnLPID);
  // resetDriveSensors = true;

  // desiredTurnValue = -24;
  // wait(0.5, sec);

  // vex::task::suspend(turnLTask);
  // enableTurnLPID = false;
  // driveL.stop();
  // driveR.stop();
  

  // roll
  driveL.spinFor(reverse, 200,degrees,false);
  driveR.spinFor(reverse, 200,degrees,false);
  wait(0.5, sec);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 150,degrees,false);
  wait(0.4, sec);
  intake.stop();

  // shoot
  // intake.spin(forward, 100, percent);
  wait(0.1, sec);
  // intake.stop();
  // wait(0.1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(forward, 100, percent);
  wait(0.12, sec);
  intake.stop();
  wait(0.3, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);
 
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
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

  // intake
  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -2000; // -2000
  dRDesiredVal = 580; // 590
  wait(0.15, sec); // 0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask(turnPID);
  desiredTurnValue = 71; //70?
  wait(0.3, sec); //0.2?
  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  dRkP = 0.008; //0.004
  dLkP = 0.008;
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -2600; //-2800
  dRDesiredVal = -2600; //-2450
  desiredTurnValue = 0; 
  wait(0.4, sec); //1

  dRkP = 0.004;
  dLkP = 0.004;
  wait(0.8, sec); //0.5

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

  vex::task::resume(spinfly);
  wait(0.45, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 0.95,turns,false);
  driveR.spinFor(forward, 0.95,turns,false);
  wait(0.25, sec);
  driveL.stop();
  driveR.stop();
  
  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -84;

  wait(0.27, sec);

  vex::task::stop(turnTask);
  enableTurnLPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.2, sec);
  

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.25, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  // intake.spin(forward, 100, percent);
  // wait(0.14, sec);
  // intake.stop();
  wait(0.1, sec);

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
  wait(0.05, sec);  

  // turn to intake
  // InertialL.setHeading(0,degrees);
  // InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 1.0,turns,false);
  driveR.spinFor(reverse, 1.0,turns,false);
  wait(0.36, sec);
  driveL.stop();
  driveR.stop();
  
  enableTurnPID = true;
  vex::task turnTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -10;

  wait(0.27, sec);

  vex::task::stop(turnTask1);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  dRkP = 0.008;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.008;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3400; 
  dRDesiredVal = -3400; 
  wait(0.4, sec);

  dRkP = 0.004;
  dLkP = 0.004;
  wait(0.7, sec);

  dRkP = 0.003;
  dLkP = 0.003;
  wait(0.3, sec);

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

  vex::task::resume(spinfly);
  // vex::task spinfly(flyPI);

  wait(0.3, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveL.spinFor(reverse,0.9,turns, false);
  driveR.setVelocity(100,percent);
  driveR.spinFor(forward,0.9,turns, false);
  wait(0.2, sec);
  driveR.stop();
  driveL.stop();

  enableTurnPID = true;
  vex::task turnRTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -115;

  wait(0.35, sec);

  vex::task::stop(turnRTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  // driveL.setVelocity(30,percent);
  // driveR.setVelocity(30,percent);
  // driveL.spinFor(forward,150,degrees,false);
  // driveR.spinFor(forward,150,degrees,false);
  // wait(0.3, sec);
  // driveL.stop();
  // driveR.stop();
  // wait(0.05, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.1, sec);

  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.1, sec);

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

void r9v1() {
  
  // starting position: straight

  angler.set(true);

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

  driveL.setStopping(coast);
  driveR.setStopping(coast);

  intake.spin(forward,12,voltageUnits::volt);

  dRkP = 0.008;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.008;
  dLkI = 0.0;
  dLkD = 0.05;
  
  // turn
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTaskA(driveLPID);
  vex::task driveRTaskA(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -1550; 
  dRDesiredVal = -2500; 
  wait(0.6, sec);
  vex::task::suspend(driveLTaskA); 
  vex::task::suspend(driveRTaskA);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  
  // wait(0.03, sec);

  wait(0.3, sec);

  // turn to roller and roll
  vex::task::resume(driveLTaskA); 
  vex::task::resume(driveRTaskA);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 1200; 
  dRDesiredVal = 450; 
  dLkP = 0.01;
  dRkP = 0.01;
  wait(0.58, sec);
  vex::task::suspend(driveLTaskA); 
  vex::task::suspend(driveRTaskA);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 200,degrees,false);
  wait(0.1, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;
  // turnKP = 0.11;
  desiredTurnValue = -24; 

  wait(0.4, sec);

  vex::task::stop(turnLTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  turnKP = 0.107;
  
  Controller1.Screen.print(InertialL.heading(degrees));
 

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.3, sec);
  intake.stop();
  wait(0.1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(forward, 100, percent);
  wait(0.12 , sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.2, sec);

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
/*
  // intake
  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.009;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -270; 
  dLDesiredVal = 270; 
  wait(0.1, sec); //0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -60;

  wait(0.25, sec);

  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  dRkP = 0.007;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.007;
  dLkI = 0.0;
  dLkD = 0.05;

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -3200; //-3100
  dLDesiredVal = -3200; //-2850
  // desiredTurnValue = 0; 
  wait(1.7, sec);

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

  vex::task::resume(spinfly);
  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.3, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveR.spinFor(reverse, 0.7,turns,false);
  driveL.spinFor(forward, 0.7,turns,false);
  wait(0.15, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnRTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 86;

  wait(0.25, sec);

  vex::task::stop(turnRTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.25, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.02, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  // intake.spin(forward, 100, percent);
  // wait(0.14, sec);
  // intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.2, sec);
  intake.stop(); 
  wait(0.1, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  wait(0.1, sec);  


  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  // turn
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 1.9,turns, false);
  driveR.spinFor(reverse, 1.9,turns, false);
  wait(0.8, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 172;    /////////////////////////////////////////////////////

  wait(0.25, sec);

  vex::task::stop(turnTask1);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  // intake
  // vex::task::resume(driveLTask1); 
  // vex::task::resume(driveRTask1);
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3000; 
  dRDesiredVal = -2700;
  dRkP = 0.007;
  dLkP = 0.007;
  wait(0.7, sec);
  dRkP = 0.005;
  dLkP = 0.005;
  wait(0.5, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.15, sec);

  // turn to shoot
  intake.spin(forward, 12, voltageUnits::volt);
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask1);
  vex::task::resume(driveRTask1);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 2650; 
  dRDesiredVal = 2950;
  dRkP = 0.008;
  dLkP = 0.008;
  wait(0.4, sec);

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

  vex::task::resume(spinfly);
  // vex::task spinfly(flyPI);

  wait(0.3, sec);

  dRkP = 0.004;
  dLkP = 0.004;
  wait(0.3, sec);
  vex::task::stop(driveLTask1); 
  vex::task::stop(driveRTask1);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 1.95,turns, false);
  driveR.spinFor(forward, 1.95,turns, false);
  wait(0.5, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask2(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -175;    

  wait(0.3, sec);

  vex::task::stop(turnTask2);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  // wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.1, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  // intake.spin(forward, 100, percent);
  // wait(0.14, sec);
  // intake.stop();
  wait(0.1, sec);

  intake.spin(reverse,100,percent);
  wait(0.25, sec);
  intake.stop(); 
  wait(0.15, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;  
*/
}

void r9v2() {
  
  // starting position: straight

  angler.set(true);

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

  driveL.setStopping(coast);
  driveR.setStopping(coast);

  intake.spin(forward,12,voltageUnits::volt);

  dRkP = 0.008;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.008;
  dLkI = 0.0;
  dLkD = 0.05;
  
  // turn
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTaskA(driveLPID);
  vex::task driveRTaskA(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -1550; 
  dRDesiredVal = -2500; 
  wait(0.6, sec);
  vex::task::suspend(driveLTaskA); 
  vex::task::suspend(driveRTaskA);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  
  // wait(0.03, sec);

  wait(0.3, sec);

  // turn to roller and roll
  vex::task::resume(driveLTaskA); 
  vex::task::resume(driveRTaskA);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 1200; 
  dRDesiredVal = 450; 
  dLkP = 0.01;
  dRkP = 0.01;
  wait(0.58, sec);
  vex::task::suspend(driveLTaskA); 
  vex::task::suspend(driveRTaskA);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 200,degrees,false);
  wait(0.1, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;
  // turnKP = 0.11;
  desiredTurnValue = -24; 

  wait(0.4, sec);

  vex::task::stop(turnLTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  turnKP = 0.107;
  
  Controller1.Screen.print(InertialL.heading(degrees));
 

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.3, sec);
  intake.stop();
  wait(0.1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(forward, 100, percent);
  wait(0.12, sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.2, sec);

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

  // intake
  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.009;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -270; 
  dLDesiredVal = 270; 
  wait(0.1, sec); //0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -60;

  wait(0.25, sec);

  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  dRkP = 0.007;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.007;
  dLkI = 0.0;
  dLkD = 0.05;

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -3100; //-3100
  dLDesiredVal = -3100; //-2850
  // desiredTurnValue = 0; 
  wait(1.7, sec);

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

  vex::task::resume(spinfly);
  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.3, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveR.spinFor(reverse, 0.7,turns,false);
  driveL.spinFor(forward, 0.7,turns,false);
  wait(0.15, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnRTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 86;

  wait(0.25, sec);

  vex::task::stop(turnRTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.25, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.02, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  // intake.spin(forward, 100, percent);
  // wait(0.14, sec);
  // intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.2, sec);
  intake.stop(); 
  wait(0.1, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  wait(0.1, sec);  



  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  // turn
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 0.5,turns, false);
  driveR.spinFor(forward, 0.5,turns, false);
  wait(0.15, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -90;

  wait(0.25, sec);

  vex::task::stop(turnTask1);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  // intake
  // vex::task::resume(driveLTask1); 
  // vex::task::resume(driveRTask1);
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3200; 
  dRDesiredVal = -3300;
  dRkP = 0.01;
  dLkP = 0.01;
  wait(0.3, sec);
  dRkP = 0.003;
  dLkP = 0.003;
  wait(0.4, sec);

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

  vex::task::resume(spinfly);
  // vex::task spinfly(flyPI);

  dRkP = 0.004;
  dLkP = 0.004;
  wait(1.05, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.15, sec);

  

  // turn to shoot
  intake.spin(forward, 12, voltageUnits::volt);
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 1.25,turns, false);
  driveR.spinFor(reverse, 1.25,turns, false);
  wait(0.5, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnTask2(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 126;

  wait(0.35, sec);

  vex::task::stop(turnTask2);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.2, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);

  // intake.spin(forward, 100, percent);
  // wait(0.14, sec);
  // intake.stop();
  // wait(0.01, sec);

  intake.spin(reverse,100,percent);
  wait(0.25, sec);
  intake.stop(); 
  wait(0.15, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;  

}

void l8win() {
  
  // starting position: facing goal

  angler.set(true);

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

  driveL.setVelocity(30,percent);
  driveR.setVelocity(30,percent);
  driveL.setStopping(coast);
  driveR.setStopping(coast);

  // roll
  intake.spin(forward,12,voltageUnits::volt);
  driveL.spinFor(reverse, 50,degrees,false);
  driveR.spinFor(reverse, 50,degrees,false);
  wait(0.15, sec);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 150,degrees,false);
  wait(0.4, sec);
  intake.stop();

  // shoot
  wait(1, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);
 
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.1, sec);

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

  // intake
  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -1900; // -2000
  dRDesiredVal = 560; // 590
  wait(0.15, sec); // 0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnRPID = true;
  vex::task turnTask(turnRightPID);
  desiredTurnValue = 68;
  wait(0.3, sec);
  vex::task::stop(turnTask);
  enableTurnRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  dRkP = 0.004;
  dLkP = 0.004;
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -2800; //-2800
  dRDesiredVal = -2800; //-2450
  desiredTurnValue = 0; 
  wait(1.5, sec); //1

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

  vex::task::resume(spinfly);
  wait(0.3, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 0.9,turns,false);
  driveR.spinFor(forward, 0.9,turns,false);
  wait(0.25, sec);
  driveL.stop();
  driveR.stop();
  
  enableTurnLPID = true;
  vex::task turnLTask(turnLeftPID);
  resetDriveSensors = true;

  desiredTurnValue = 274;

  wait(0.27, sec);

  vex::task::stop(turnLTask);
  enableTurnLPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);
  

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.25, sec);
  intake.stop();
  wait(0.07, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.12, sec);

  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.12, sec);

  intake.spin(reverse,100,percent);
  wait(0.25, sec);
  intake.stop(); 
  wait(0.1, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;
  wait(0.1, sec);  

  // turn to intake
  InertialL.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 0.77,turns,false);
  driveR.spinFor(reverse, 0.77,turns,false);
  wait(0.22, sec);
  driveL.stop();
  driveR.stop();
  
  enableTurnRPID = true;
  vex::task turnTask1(turnRightPID);
  resetDriveSensors = true;

  desiredTurnValue = 79;

  wait(0.3, sec);

  vex::task::stop(turnTask1);
  enableTurnRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  dRkP = 0.008;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.008;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3300; 
  dRDesiredVal = -3300; 
  wait(0.3, sec);

  dRkP = 0.004;
  dLkP = 0.004;
  wait(0.7, sec);

  dRkP = 0.003;
  dLkP = 0.003;
  wait(0.4, sec);

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

  vex::task::resume(spinfly);
  // vex::task spinfly(flyPI);

  wait(0.2, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  InertialL.setHeading(0,degrees);
  // InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveL.spinFor(reverse,0.92,turns, false);
  driveR.setVelocity(100,percent);
  driveR.spinFor(forward,0.92,turns, false);
  wait(0.23, sec);
  driveR.stop();
  driveL.stop();

  enableTurnLPID = true;
  vex::task turnRTask(turnLeftPID);
  resetDriveSensors = true;

  desiredTurnValue = 249;

  wait(0.25, sec);

  vex::task::stop(turnRTask);
  enableTurnLPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.14, sec);

  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.12, sec);

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

  // go to roller
  driveL.spinFor(reverse,0.9,turns,false);
  driveR.spinFor(forward,0.9,turns,false);
  wait(0.5, sec);

  intake.spin(forward,10,voltageUnits::volt);

  dRkP = 0.007;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.007;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask2(driveLPID);
  vex::task driveRTask2(driveRPID);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 3500; 
  dRDesiredVal = 3500; 
  wait(0.5, sec);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 900; 
  dRDesiredVal = 2000; 
  wait(0.5, sec);

  vex::task::suspend(driveLTask2); 
  vex::task::suspend(driveRTask2);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  driveL.setVelocity(30,percent);
  driveR.setVelocity(30,percent);
  driveL.spinFor(reverse,2,turns,false);
  driveR.spinFor(reverse,2,turns,false);
  wait(0.5, sec);
  driveL.spinFor(forward,100,degrees,false);
  driveR.spinFor(forward,100,degrees,false);
  wait(0.3, sec);
  driveL.stop();
  driveR.stop();

}


void r8v1() {
  // DONE 4/24
  // starting position: straight

  angler.set(true);

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

  driveL.setStopping(coast);
  driveR.setStopping(coast);

  intake.spin(forward,12,voltageUnits::volt);

  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.009;
  dLkI = 0.0;
  dLkD = 0.05;
  
  // turn
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,900,degrees, false);
  driveR.spinFor(forward,900,degrees,false);
  wait(0.7, sec);
  // enableDriveLPID = true;
  // enableDriveRPID = true;
  // vex::task driveLTaskA(driveLPID);
  // vex::task driveRTaskA(driveRPID);
  // resetDriveLSensors = true;
  // resetDriveRSensors = true;
  // dLDesiredVal = -1700; 
  // dRDesiredVal = -1700; 
  // wait(0.5, sec);
  // vex::task::suspend(driveLTaskA); 
  // vex::task::suspend(driveRTaskA);
  // driveL.setStopping(brake);
  // driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  
  // wait(0.03, sec);

  wait(0.3, sec);

  // turn to roller and roll
  // vex::task::resume(driveLTaskA); 
  // vex::task::resume(driveRTaskA);
  vex::task driveLTaskA(driveLPID);
  vex::task driveRTaskA(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 850; 
  dRDesiredVal = -850; 
  dLkP = 0.01;
  dRkP = 0.01;
  wait(0.4, sec);
  vex::task::suspend(driveLTaskA); 
  vex::task::suspend(driveRTaskA);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 0.76,turns,false);
  driveR.spinFor(reverse, 0.76,turns,false);
  wait(0.35, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 150,degrees,false);
  wait(0.1, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;
  // turnKP = 0.11;
  desiredTurnValue = -84; //-80

  wait(0.3, sec);

  vex::task::stop(turnLTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  turnKP = 0.107;
  
  Controller1.Screen.print(InertialL.heading(degrees));
 

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.12 , sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.4, sec);

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
/*
  // intake
  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.009;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -580; 
  dLDesiredVal = 580; 
  wait(0.4, sec); //0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -59;

  wait(0.25, sec);

  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  dRkP = 0.007;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.007;
  dLkI = 0.0;
  dLkD = 0.05;

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -3300; //-3100
  dLDesiredVal = -3300; //-2850
  // desiredTurnValue = 0; 
  wait(1.75, sec);

  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.15, sec);

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

  vex::task::resume(spinfly);
  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.1, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveR.spinFor(reverse, 0.88,turns,false);
  driveL.spinFor(forward, 0.88,turns,false);
  wait(0.25, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnRTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 94;

  wait(0.25, sec);

  vex::task::stop(turnRTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  // wait(0.1, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.15, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

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
  wait(0.1, sec);  


  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
 
  // turn
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 1.8,turns, false);
  driveR.spinFor(reverse, 1.8,turns, false);
  wait(0.8, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 173;    /////////////////////////////////////////////////////

  wait(0.25, sec);

  vex::task::stop(turnTask1);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  // intake
  // vex::task::resume(driveLTask1); 
  // vex::task::resume(driveRTask1);
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3000; 
  dRDesiredVal = -2700;
  dRkP = 0.007;
  dLkP = 0.007;
  wait(0.7, sec);
  dRkP = 0.005;
  dLkP = 0.005;
  wait(0.5, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.15, sec);

  // turn to shoot
  intake.spin(forward, 12, voltageUnits::volt);
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask1);
  vex::task::resume(driveRTask1);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 2650; 
  dRDesiredVal = 2950;
  dRkP = 0.008;
  dLkP = 0.008;
  wait(0.7, sec);

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

  vex::task::resume(spinfly);
  // vex::task spinfly(flyPI);

  // wait(0.2, sec);

  dRkP = 0.004;
  dLkP = 0.004;
  wait(0.3, sec);
  vex::task::stop(driveLTask1); 
  vex::task::stop(driveRTask1);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 1.82,turns, false);
  driveR.spinFor(forward, 1.82,turns, false);
  wait(0.55, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask2(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -168;    

  wait(0.5, sec);

  vex::task::stop(turnTask2);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.05, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  // wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.35, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.25, sec);

  intake.spin(reverse,100,percent);
  wait(0.25, sec);
  intake.stop(); 
  wait(0.15, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;  
*/
}


void r8v2() {
  // 
  // starting position: straight

  angler.set(true);

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

  driveL.setStopping(coast);
  driveR.setStopping(coast);

  intake.spin(forward,12,voltageUnits::volt);

  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.009;
  dLkI = 0.0;
  dLkD = 0.05;
  
  // turn
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTaskA(driveLPID);
  vex::task driveRTaskA(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -1700; 
  dRDesiredVal = -1700; 
  wait(0.5, sec);
  vex::task::suspend(driveLTaskA); 
  vex::task::suspend(driveRTaskA);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  
  // wait(0.03, sec);

  wait(0.3, sec);

  // turn to roller and roll
  vex::task::resume(driveLTaskA); 
  vex::task::resume(driveRTaskA);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 850; 
  dRDesiredVal = -850; 
  dLkP = 0.01;
  dRkP = 0.01;
  wait(0.4, sec);
  vex::task::suspend(driveLTaskA); 
  vex::task::suspend(driveRTaskA);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 0.76,turns,false);
  driveR.spinFor(reverse, 0.76,turns,false);
  wait(0.35, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 150,degrees,false);
  driveR.spinFor(forward, 150,degrees,false);
  wait(0.1, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;
  // turnKP = 0.11;
  desiredTurnValue = -80; //-80

  wait(0.3, sec);

  vex::task::stop(turnLTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  turnKP = 0.107;
  
  Controller1.Screen.print(InertialL.heading(degrees));
 

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.12 , sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.4, sec);

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

  // intake
  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.009;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -570; 
  dLDesiredVal = 570; 
  wait(0.4, sec); //0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -56;

  wait(0.25, sec);

  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  dRkP = 0.007;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.007;
  dLkI = 0.0;
  dLkD = 0.05;

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -3250; //-3100
  dLDesiredVal = -3250; //-2850
  // desiredTurnValue = 0; 
  wait(1.75, sec);

  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.15, sec);

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

  vex::task::resume(spinfly);
  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.1, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveR.spinFor(reverse, 0.85,turns,false);
  driveL.spinFor(forward, 0.85,turns,false);
  wait(0.25, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnRTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 92;

  wait(0.25, sec);

  vex::task::stop(turnRTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  // wait(0.1, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.15, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

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
  wait(0.1, sec);  


  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  // turn
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 0.5,turns, false);
  driveR.spinFor(forward, 0.5,turns, false);
  wait(0.15, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  enableTurnPID = true;
  vex::task turnTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -90;

  wait(0.25, sec);

  vex::task::stop(turnTask1);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  intake.spin(forward, 12, voltageUnits::volt);

  // intake
  // vex::task::resume(driveLTask1); 
  // vex::task::resume(driveRTask1);
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3200; 
  dRDesiredVal = -3300;
  dRkP = 0.01;
  dLkP = 0.01;
  wait(0.28, sec);
  dRkP = 0.002;
  dLkP = 0.002;
  wait(0.5, sec);

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

  vex::task::resume(spinfly);
  // vex::task spinfly(flyPI);

  dRkP = 0.004;
  dLkP = 0.004;
  wait(1, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.15, sec);

  

  // turn to shoot
  intake.spin(forward, 12, voltageUnits::volt);
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);

  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(forward, 1.34,turns, false);
  driveR.spinFor(reverse, 1.34,turns, false);
  wait(0.5, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnTask2(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 128;

  wait(0.5, sec);

  vex::task::stop(turnTask2);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.2, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.2, sec);
  intake.stop();
  wait(0.0 , sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.1, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.1, sec);

  intake.spin(reverse,100,percent);
  wait(0.25, sec);
  intake.stop(); 
  wait(0.15, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;  

}


void test1() {
  
  //  turnKP = 0.02; // 0.2
  //  turnKI = 0.000;
  //  turnKD = 0.002; // 0.05

  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);

  driveL.spin(forward,100,percent);
  driveR.spin(reverse,100,percent);
  wait(0.3, sec);
  driveL.stop();
  driveR.stop();
  wait(1, sec);

  enableTurnPID = true;
  vex::task turnLTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 90;

  wait(0.5, sec);

  vex::task::stop(turnLTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  printf("L:%f   R:%f\n", InertialL.heading(degrees), InertialR.heading(degrees));

}
//////////////////////////////////////////////////////////////
void l5v1() {
  // shoot
  angler.set(true);

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

  wait(1.8, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.12 , sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.3, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.4, sec);

  intake.spin(reverse,80,percent);
  wait(0.35, sec);
  intake.stop(); 
  wait(0.1, sec);
  vex::task::suspend(spinfly); 

  fkP = 0;
  fkI = 0;
  fkD = 0;
  fDesiredVal = 0; 
  flywheel.stop();
  enableFlywheelPID = false;

  // roll
  intake.spin(forward,8,voltageUnits::volt);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(reverse,150,degrees,false);
  driveR.spinFor(reverse,150,degrees,false);
  wait(0.4, sec);
  driveL.stop();
  driveR.stop();
  driveL.spinFor(forward,200,degrees,false);
  driveR.spinFor(forward,200,degrees,false);
  wait(0.3, sec);
  driveL.stop();
  driveR.stop();

  // turn to intake
  dRkP = 0.01;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.01;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  // intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -2000; // -2000
  dRDesiredVal = 590; // 590
  wait(0.15, sec); // 0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  enableTurnPID = true;
  vex::task turnTask(turnPID);
  desiredTurnValue = 73; //70?
  wait(0.5, sec); //0.2?
  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  dRkP = 0.004; //0.004
  dLkP = 0.004;
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = -3200; //-2800
  dRDesiredVal = -3200; //-2450
  desiredTurnValue = 0; 
  wait(1, sec); //1
  intake.spin(forward,12,voltageUnits::volt);

  dRkP = 0.003;
  dLkP = 0.003;
  wait(0.9, sec); //0.5

  enableFlywheelPID = true;
  // reset
  fError = 0;
  fPrevError = 0;
  fDeriv = 0;
  fTotalError = 0;

  fDesiredVal = 11;
  fkP = 0.48; //0.38
  fkI = 0.1; //0.1
  fkD = 0.0; //0.06

  vex::task::resume(spinfly);
  wait(0.2, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveL.spinFor(reverse, 0.95,turns,false);
  driveR.spinFor(forward, 0.95,turns,false);
  wait(0.4, sec);
  driveL.stop();
  driveR.stop();
  
  enableTurnPID = true;
  vex::task turnLTask1(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -88;

  wait(0.5, sec);

  vex::task::stop(turnLTask1);
  enableTurnLPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.2, sec);
  

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.05, sec);

  fDesiredVal = 12;

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.15, sec);

  intake.spin(reverse,100,percent);
  wait(0.14, sec);
  intake.stop(); 
  wait(0.05, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.15, sec);

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

  driveR.spinFor(reverse,250,degrees);
  wait(0.25, sec);

  //
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);

  dRkP = 0.008;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.008;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask1(driveLPID);
  vex::task driveRTask1(driveRPID);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dLDesiredVal = 1800; 
  dRDesiredVal = 2400; 
  wait(0.9, sec);
  vex::task::suspend(driveLTask1); 
  vex::task::suspend(driveRTask1);
  driveL.setStopping(brake);
  driveR.setStopping(brake);
  driveL.stop();
  driveR.stop();
  // wait(0.4, sec);

  // driveL.setVelocity(100,percent);
  // driveL.spinFor(forward,0.1,turns, false);
  // driveR.setVelocity(100,percent);
  // driveR.spinFor(reverse,0.1,turns, false);
  // wait(0.1, sec);
  // driveR.stop();
  // driveL.stop();

  // enableTurnPID = true;
  // vex::task turnRTask(turnPID);
  // resetDriveSensors = true;

  // desiredTurnValue = 13;

  // wait(0.2, sec);

  // vex::task::stop(turnRTask);
  // enableTurnPID = false;
  // driveL.stop();
  // driveR.stop();
  // wait(0.05, sec);

  

  // enableDriveLPID = true;
  // enableDriveRPID = true;
  // vex::task driveLTask2(driveLPID);
  // vex::task driveRTask2(driveRPID);
  // resetDriveLSensors = true;
  // resetDriveRSensors = true;
  // intake.spin(forward,12,voltageUnits::volt);
  // dRkP = 0.002;
  // dLkP = 0.002;
  // resetDriveLSensors = true;
  // resetDriveRSensors = true;
  // dLDesiredVal = -3200; 
  // dRDesiredVal = -2800;
  // desiredTurnValue = 0; 
  // wait(0.7, sec);

  // // enableFlywheelPID = true;
  // // // reset
  // // fError = 0;
  // // fPrevError = 0;
  // // fDeriv = 0;
  // // fTotalError = 0;

  // // fDesiredVal = 12;
  // // fkP = 0.48; //0.38
  // // fkI = 0.1; //0.1
  // // fkD = 0.0; //0.06

  // // vex::task::resume(spinfly);
  // // vex::task spinfly(flyPI);

  // // dLDesiredVal = -2800;
  // dRkP = 0.003;
  // dLkP = 0.003;
  // wait(0.8, sec);

  // vex::task::stop(driveLTask2); 
  // vex::task::stop(driveRTask2);
  // enableDriveLPID = false;
  // enableDriveRPID = false;
  // driveL.stop();
  // driveR.setStopping(hold);
  // driveR.stop();
  // wait(0.5, sec);

  // // turn to shoot
  // // InertialL.setHeading(0,degrees);
  // // InertialR.setHeading(0,degrees);
  // driveL.setVelocity(100,percent);
  // driveL.spinFor(reverse, 1.0,turns,false);
  // // driveR.setVelocity(90,percent);
  // // driveR.spinFor(forward, 0.2,turns,false);
  // wait(0.4, sec);
  // driveL.stop();
  // driveR.stop();

  // enableTurnPID = true;
  // vex::task turnLTask2(turnPID);
  // resetDriveSensors = true;
  // turnKP = 0.13;
  // desiredTurnValue = -50;

  // wait(0.5, sec);

  // vex::task::stop(turnLTask2);
  // enableTurnPID = false;
  // driveL.stop();
  // driveR.stop();

  // shoot
  // intake.spin(forward, 100, percent);
  // wait(0.05, sec);
  // intake.stop();
  // wait(0.1, sec);
  
  // intake.spin(reverse,100,percent);
  // wait(0.14, sec);
  // intake.stop();
  // wait(0.01, sec);

  // intake.spin(forward, 100, percent);
  // wait(0.1, sec);
  // intake.stop();
  // wait(0.15, sec);

  // intake.spin(reverse,100,percent);
  // wait(0.14, sec);
  // intake.stop(); 
  // wait(0.01, sec);

  // intake.spin(forward, 100, percent);
  // wait(0.1, sec);
  // intake.stop();
  // wait(0.1, sec);

  // intake.spin(reverse,100,percent);
  // wait(0.2, sec);
  // intake.stop(); 
  // wait(0.1, sec);
  // vex::task::suspend(spinfly); 

  // fkP = 0;
  // fkI = 0;
  // fkD = 0;
  // fDesiredVal = 0; 
  // flywheel.stop();
  // enableFlywheelPID = false; 
}

void r5v1() {
  // shoot
  angler.set(true);

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

  driveL.spinFor(forward,300,degrees, false);
  driveR.spinFor(forward,300,degrees, false);

  wait(1.9, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.12 , sec);
  intake.stop();
  wait(0.2, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.1, sec);

  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.45, sec);

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

  // go to roll
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(forward,350,degrees,false);
  driveR.spinFor(reverse,350,degrees,false);
  wait(0.4, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  // enableTurnPID = true;
  // vex::task turnTaskA(turnPID);
  // resetDriveSensors = true;
  // desiredTurnValue = 60; //
  // wait(0.3, sec); //
  // vex::task::stop(turnTaskA);
  // enableTurnPID = false;
  // driveL.stop();
  // driveR.stop();
  // wait(0.1, sec);

  driveL.spinFor(forward,900,degrees,false);
  driveR.spinFor(forward,900,degrees,false);
  wait(0.9, sec);

  driveL.spinFor(reverse,0.9,turns,false);
  driveR.spinFor(forward,0.9,turns,false);
  wait(0.9, sec);
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  // roll
  intake.spin(forward,7,voltageUnits::volt);
  driveL.setVelocity(50,percent);
  driveR.setVelocity(50,percent);
  driveL.spinFor(reverse,450,degrees,false);
  driveR.spinFor(reverse,450,degrees,false);
  wait(0.4, sec);
  driveL.stop();
  driveR.stop();
  driveL.spinFor(forward,250,degrees,false);
  driveR.spinFor(forward,250,degrees,false);
  wait(0.3, sec);
  driveL.stop();
  driveR.stop();

  // turn to intake
  dRkP = 0.009;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.009;
  dLkI = 0.0;
  dLkD = 0.05;
  
  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task driveLTask(driveLPID);
  vex::task driveRTask(driveRPID);
  intake.spin(forward,12,voltageUnits::volt);

  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -510; 
  dLDesiredVal = 510; 
  wait(0.4, sec); //0.183
  vex::task::suspend(driveLTask); 
  vex::task::suspend(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);
  
  enableTurnPID = true;
  vex::task turnTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = -50;

  wait(0.4, sec);

  vex::task::stop(turnTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.05, sec);

  dRkP = 0.007;
  dRkI = 0.0;
  dRkD = 0.05;
  dLkP = 0.007;
  dLkI = 0.0;
  dLkD = 0.05;

  enableDriveLPID = true;
  enableDriveRPID = true;
  vex::task::resume(driveLTask);
  vex::task::resume(driveRTask);
  resetDriveLSensors = true;
  resetDriveRSensors = true;
  dRDesiredVal = -2200; //-3100
  dLDesiredVal = -2200; //-2850
  // desiredTurnValue = 0; 
  wait(1.6, sec);

  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.15, sec);

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

  vex::task::resume(spinfly);
  dRkP = 0.009;
  dLkP = 0.009;
  wait(0.1, sec); //0.7

  vex::task::stop(driveLTask); 
  vex::task::stop(driveRTask);
  enableDriveLPID = false;
  enableDriveRPID = false;
  driveL.stop();
  driveR.stop();
  wait(0.1, sec);

  // turn to shoot
  InertialL.setHeading(0,degrees);
  InertialR.setHeading(0,degrees);
  driveL.setVelocity(100,percent);
  driveR.setVelocity(100,percent);
  driveR.spinFor(reverse, 0.88,turns,false);
  driveL.spinFor(forward, 0.88,turns,false);
  wait(0.25, sec);
  driveL.stop();
  driveR.stop();

  enableTurnPID = true;
  vex::task turnRTask(turnPID);
  resetDriveSensors = true;

  desiredTurnValue = 94;

  wait(0.25, sec);

  vex::task::stop(turnRTask);
  enableTurnPID = false;
  driveL.stop();
  driveR.stop();
  // wait(0.1, sec);

  // shoot
  intake.spin(forward, 100, percent);
  wait(0.1, sec);
  intake.stop();
  wait(0.01, sec);
  
  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop();
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.15, sec);

  intake.spin(reverse,100,percent);
  wait(0.15, sec);
  intake.stop(); 
  wait(0.01, sec);

  intake.spin(forward, 100, percent);
  wait(0.14, sec);
  intake.stop();
  wait(0.2, sec);

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
//////////////////////////////////////////////////////////////

void autonomous(void) {
  // enableDriver = false;
  // auto2v1();
  // test1();
  // l8win();
  // r8v1();
  l5v1();

}

void testExpansion()
{
  Brain.Screen.print("expand");
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
    
    Controller1.Axis3.changed(driver);
    Controller1.Axis1.changed(driver);
    // Controller1.Axis4.changed(tank);
    Controller1.ButtonB.pressed(ind);
    // Controller1.ButtonR2.pressed(tripShot1);
    // Controller1.ButtonR2.pressed(tripShot3);
    Controller1.ButtonR2.pressed(tripShotC);
    // Controller1.ButtonUp.pressed(expansion);
    Controller1.ButtonX.pressed(tripShotA);
    Controller1.ButtonA.pressed(tripShotB);

    // Controller1.ButtonB.pressed(setCoast);
    // Controller1.ButtonY.pressed(setBrake);
    // Controller1.ButtonX.pressed(setHold);

    // Controller1.ButtonR1.pressed(testFwd);
    // Controller1.ButtonR2.pressed(testBwd);
    
    Controller1.ButtonDown.pressed(temp);
    Controller1.ButtonUp.pressed(expansion);
    Controller2.ButtonUp.pressed(expansion);

    wait(100, msec);
  }
}
