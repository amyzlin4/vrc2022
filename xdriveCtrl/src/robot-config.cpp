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