#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Motor1FL = motor(PORT1, ratio18_1, false);
motor Motor2FR = motor(PORT2, ratio18_1, true);
motor Motor3BL = motor(PORT3, ratio18_1, false);
motor Motor4BR = motor(PORT4, ratio18_1, true);
controller Controller1 = controller(primary);
inertial Inertial15 = inertial(PORT15);
encoder EncoderR = encoder(Brain.ThreeWirePort.A);
encoder EncoderL = encoder(Brain.ThreeWirePort.C);
/*vex-vision-config:begin*/
signature Vision7__DUCK = signature (1, 1153, 1903, 1528, -4991, -4379, -4685, 2.8, 0);
signature Vision7__R = signature (2, 7765, 13653, 10709, -1947, -139, -1043, 1.9, 0);
signature Vision7__B = signature (3, -3791, -2949, -3370, 9719, 11889, 10804, 3.4, 0);
vision Vision7 = vision (PORT7, 50, Vision7__DUCK, Vision7__R, Vision7__B);
/*vex-vision-config:end*/
distance DistanceS = distance(PORT5);
optical OpticalS = optical(PORT14);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}