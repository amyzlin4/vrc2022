/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       me                                                        */
/*    Created:      Jul 16 2022                                               */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Motor1FL             motor         1               
// Motor2FR             motor         2               
// Motor3BL             motor         3               
// Motor4BR             motor         4               
// Controller1          controller                    
// Inertial15           inertial      15              
// EncoderR             encoder       A, B            
// EncoderL             encoder       C, D            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
double angleT = 0;
double driveBase = 10;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial15.calibrate();
  // clear encoders
}
// =================================================================================
void drive1() {
  Motor1FL.setVelocity((Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent) + Controller1.Axis1.position(percent)), percent);
  Motor2FR.setVelocity((Controller1.Axis3.position(percent) - Controller1.Axis4.position(percent) - Controller1.Axis1.position(percent)), percent);
  Motor3BL.setVelocity((Controller1.Axis3.position(percent) - Controller1.Axis4.position(percent) + Controller1.Axis1.position(percent)), percent);
  Motor4BR.setVelocity((Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent) - Controller1.Axis1.position(percent)), percent);
  Motor1FL.spin(forward);
  Motor2FR.spin(forward);
  Motor3BL.spin(forward);
  Motor4BR.spin(forward);
}
// =================================================================================
void usercontrol(void) {
  while (1) {

    // Sleep the task for a short amount of time to prevent wasted resources
    wait(20, msec); 
  }
}
// =================================================================================
void driveFor(double dist, double dir, double vel) {
  // 1 rotation = 4 * M_PI / sqrt(2) = 8.887
  // distance in inches = 1 / rotation - 0.1125
  Motor1FL.setVelocity(vel,percent);
  Motor2FR.setVelocity(vel,percent);
  Motor3BL.setVelocity(vel,percent);
  Motor4BR.setVelocity(vel,percent);
  Motor1FL.spinFor(forward, (dist * 0.0587 * (cos(dir * M_PI / 180) + sin(dir * M_PI / 180))), turns, false);
  Motor2FR.spinFor(forward, (dist * 0.0587 * (sin(dir * M_PI / 180) - cos(dir * M_PI / 180))), turns, false);
  Motor3BL.spinFor(forward, (dist * 0.0587 * (sin(dir * M_PI / 180) - cos(dir * M_PI / 180))), turns, false);
  Motor4BR.spinFor(forward, (dist * 0.0587 * (cos(dir * M_PI / 180) + sin(dir * M_PI / 180))), turns, false);
  
}
// =================================================================================
void encoderTest() {
  // Brain.Screen.print("R: ", EncoderR.position(degrees));
  // Brain.Screen.newLine();
  Brain.Screen.print("L: ", EncoderL.position(degrees));
  Brain.Screen.newLine();
  /*
  if(EncoderL.position(degrees) != EncoderR.position(degrees)) {
    double dif = (EncoderR.position(degrees) - EncoderL.position(degrees));
    angleT = asin(dif / driveBase);
    angleT = angleT * 180 / M_PI;
    Brain.Screen.print(angleT);
    Brain.Screen.newLine();
  } else {
    Brain.Screen.print("0");
    Brain.Screen.newLine();
  }
  */
}
void driveTest() {
  driveFor(24,180,50);
  wait(2,seconds);
  // driveFor(12,270,50);
  // wait(2,seconds);
  // driveFor(12,0,50);
  // wait(2,seconds);
  // driveFor(12,90,50);
  // wait(2,seconds);
}
void clearEncoders() {
  EncoderL.setPosition(0,degrees);
  Controller1.Screen.print(EncoderL.position(degrees));
  //EncoderR.setPosition(0,degrees);
  Brain.Screen.print(EncoderR.position(degrees));

}

// =================================================================================
int whenStarted() {
  Brain.Screen.print("Start");
  Brain.Screen.newLine();
  EncoderL.setPosition(0,degrees);
  //EncoderR.setPosition(0,degrees);
  Motor1FL.setVelocity(100,percent);
  Motor2FR.setVelocity(100,percent);
  Motor3BL.setVelocity(100,percent);
  Motor4BR.setVelocity(100,percent);

  return 0;
}
// =================================================================================
void autonomous(void) {
  // code
  driveTest();
}
// =================================================================================
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    Controller1.ButtonX.pressed(encoderTest);
    Controller1.ButtonY.pressed(driveTest);
    Controller1.ButtonA.pressed(clearEncoders);
    Controller1.Axis3.changed(drive1);
    Controller1.Axis4.changed(drive1);
    Controller1.Axis1.changed(drive1);
    wait(100, msec);
  }
  whenStarted();
}
// =================================================================================
// ---------------------------------------------------------------------------------
// =================================================================================
/* 
Notes: 
>> arctan = atan()
>> pi = M_PI
>> put "when Controller etc" blocks under main() 
>> when asked to travel 18 in, travels 34.5 in
>> old constant = 0.1125
>> new constant = 0.0587
>> 07/19/2022: only Left Encoder installed on robot; encoder functions haven't been tested
*/