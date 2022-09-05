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
// Vision7              vision        7               
// DistanceS            distance      5               
// OpticalS             optical       14              
// RotationS            rotation      6               
// ---- END VEXCODE CONFIGURED DEVICES ----


/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       me                                                        */
/*    Created:      Jul 16 2022                                               */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
double angleT = 0;
double driveBase = 10; // 20 inches
double degOinch = 0.057;

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
  // drive for desired distance (inches) in desired direction (angle, degrees) at desired velocity (%)
  Motor1FL.setVelocity(vel,percent);
  Motor2FR.setVelocity(vel,percent);
  Motor3BL.setVelocity(vel,percent);
  Motor4BR.setVelocity(vel,percent);
  Motor1FL.spinFor(forward, (dist * degOinch * (cos(dir * M_PI / 180) + sin(dir * M_PI / 180))), turns, false);
  Motor2FR.spinFor(forward, (dist * degOinch * (sin(dir * M_PI / 180) - cos(dir * M_PI / 180))), turns, false);
  Motor3BL.spinFor(forward, (dist * degOinch * (sin(dir * M_PI / 180) - cos(dir * M_PI / 180))), turns, false);
  Motor4BR.spinFor(forward, (dist * degOinch * (cos(dir * M_PI / 180) + sin(dir * M_PI / 180))), turns, false);
}
// =================================================================================
void turnFor(double angl, double vel) {
  // to be tested 
  double degOangl = 5;
  Motor1FL.setVelocity(vel,percent);
  Motor2FR.setVelocity(vel * -1,percent);
  Motor3BL.setVelocity(vel,percent);
  Motor4BR.setVelocity(vel * -1,percent);
  Motor1FL.spinFor(forward,angl * degOangl,degrees, false);
  Motor2FR.spinFor(forward,angl * degOangl,degrees, false);
  Motor3BL.spinFor(forward,angl * degOangl,degrees, false);
  Motor4BR.spinFor(forward,angl * degOangl,degrees, false);
}
// =================================================================================
// =================================================================================
// TESTING FUNCTIONS
void encoderTest() {
  // to be tested 
  float ER = EncoderR.position(degrees) * -1;
  float EL = EncoderL.position(degrees) * -1;
  printf("R: %f\n", ER);
  printf("L: %f\n", EL);
  // fix driveBase variable 
  if(EL != ER) {
    double dif = (ER - EL);
    angleT = asin(dif / driveBase) * 180 / M_PI;
    printf("angle=%f\n", angleT);
  } else {
    printf("0\n");
  }
}
void driveTest() {
  driveFor(24,180,50); // left 24 inches
  wait(2,seconds);
  driveFor(24,270,50); // down 24 inches
  wait(2,seconds);
  driveFor(24,0,50); // right 24 inches
  wait(2,seconds);
  driveFor(24,90,50); // up 24 inches
  wait(2,seconds);
  driveFor(34,225,50); // down/left 34 (approx 24*sqrt(2)) inches
  wait(2,seconds);
}
void driveTest2() {
  driveFor(20,90,50); // up 20 inches
  wait(3,seconds);
}
void turnTest() {
  turnFor(90,50);
  wait(2,seconds);
}
void clearEncoders() {
  // set both encoders to 0
  EncoderL.setPosition(0,degrees);
  EncoderR.setPosition(0,degrees);
  printf("R: %f\n", EncoderR.position(degrees));
  printf("L: %f\n", EncoderL.position(degrees));

}
void visionSens() {
  // to be tested 
  Vision7.takeSnapshot(Vision7__R);
  if(Vision7.objects[0].exists) {
    printf("RED\n");
  } else {
    Vision7.takeSnapshot(Vision7__B);
    if(Vision7.objects[0].exists) {
      printf("BLUE\n");
    } else {
      Vision7.takeSnapshot(Vision7__DUCK);
      if(Vision7.objects[0].exists) {
        printf("YELLOW/n");
      } else {
        printf("N/n");
      }
    }
  }
}
void goToY() {
  // to be tested 
  Vision7.takeSnapshot(Vision7__DUCK);
  double b = Vision7.objects[0].centerX;
  double c = Vision7.objects[0].centerY;
  double a = sqrt((b * b) + (c * c));
  printf("b=%f/nc=%f/na=%f/n", b, c, a);
  double ang = asin(b / a) * 180 / M_PI;
  if(b < 0) {
    ang = 180 + ang;
  } else if(c < 0) {
    ang = ang + 360;
  }
  if(ang >= 360) {
    ang = ang - 360;
  }
  printf("angle=%a/n", ang);
  driveFor(a,ang,50);
}
// distance sensor testing
void goToDis() {
  // travel to detected object
  double dis = DistanceS.objectDistance(inches);
  printf("%f\n", dis);
  double spacer = 5; // stops approx 1 inch from wall
  driveFor(dis - spacer,90,50);
}
void speedCtrl() {
  // travel to detected object; reduce velocity as robot gets closer 
  double Cdis = DistanceS.objectDistance(inches) * 4;
  while(Cdis >= 14) {
    Motor1FL.setVelocity(Cdis,percent);
    Motor2FR.setVelocity(Cdis,percent);
    Motor3BL.setVelocity(Cdis,percent);
    Motor4BR.setVelocity(Cdis,percent);
    Motor1FL.spin(forward);
    Motor2FR.spin(forward);
    Motor3BL.spin(forward);
    Motor4BR.spin(forward);
    Cdis = DistanceS.objectDistance(inches) * 4;
  }
  Motor1FL.stop();
  Motor2FR.stop(); 
  Motor3BL.stop();
  Motor4BR.stop();
}
void colorS() {
  // prints hue value
  OpticalS.setLight(ledState::on);
  OpticalS.setLightPower(50,percent);
  printf("Hue: %f\n", OpticalS.hue());
  OpticalS.setLight(ledState::off);
}
void RB() {
  // recognizes red and blue objects 
  OpticalS.setLight(ledState::on);
  OpticalS.setLightPower(50,percent);
  if(OpticalS.hue() < 40) {
    printf("RED\n");
  } else if(OpticalS.hue() > 180 && OpticalS.hue() < 315) {
    printf("BLUE\n");
  }
  OpticalS.setLight(ledState::off);
}
// =================================================================================
// =================================================================================
int whenStarted() {
  Brain.Screen.print("Start");
  Brain.Screen.newLine();
  printf("Start\n");
  EncoderL.setPosition(0,degrees);
  EncoderR.setPosition(0,degrees);
  Motor1FL.setVelocity(100,percent);
  Motor2FR.setVelocity(100,percent);
  Motor3BL.setVelocity(100,percent);
  Motor4BR.setVelocity(100,percent);

  return 0;
}
// =================================================================================
void autonomous(void) {
  // code
  
}
// =================================================================================
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  whenStarted();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    Controller1.ButtonX.pressed(encoderTest);
    Controller1.ButtonY.pressed(driveTest2);
    Controller1.ButtonA.pressed(colorS);
    Controller1.ButtonB.pressed(turnTest);
    Controller1.ButtonRight.pressed(goToY);
    Controller1.ButtonLeft.pressed(goToDis);
    Controller1.ButtonUp.pressed(speedCtrl);
    Controller1.Axis3.changed(drive1);
    Controller1.Axis4.changed(drive1);
    Controller1.Axis1.changed(drive1);
    wait(100, msec);
  }
}
// =================================================================================
// ---------------------------------------------------------------------------------
// =================================================================================
/* 
Notes: 
>> arctan = atan()
>> pi = M_PI
>> put "when Controller etc" blocks under main() while loop
>> driveFor function:
>>>> when asked to travel 18 in, travels 34.5 in
>>>> old constant = 0.1125
>>>> new constant = 0.057
>> printf to print to terminal (using %f etc for formating and "\n" for new line)
>> clear terminal manually with trash icon 
>> distance between sensor and front of robot = 2 inches
>> distance sensor may require 3-10 inches of error to react 
>> optical sensor works best at around 50% light
*/