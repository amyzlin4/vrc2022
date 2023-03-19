#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor intake = motor(PORT3, ratio6_1, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT1);
motor MotorR1 = motor(PORT8, ratio6_1, false);
motor MotorR2 = motor(PORT9, ratio6_1, false);
motor MotorR3 = motor(PORT10, ratio6_1, false);
motor flywheel = motor(PORT2, ratio6_1, false);
digital_out angler = digital_out(Brain.ThreeWirePort.A);
digital_out expand = digital_out(Brain.ThreeWirePort.B);
motor MotorL1 = motor(PORT11, ratio6_1, true);
motor MotorL2 = motor(PORT12, ratio6_1, true);
motor MotorL3 = motor(PORT13, ratio6_1, true);

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