#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor intake = motor(PORT3, ratio6_1, true);
controller Controller1 = controller(primary);
digital_out roller = digital_out(Brain.ThreeWirePort.H);
digital_out expand = digital_out(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT19);
motor MotorR1 = motor(PORT13, ratio6_1, true);
motor MotorR2 = motor(PORT12, ratio6_1, false);
motor MotorR3 = motor(PORT11, ratio6_1, false);
motor MotorL1 = motor(PORT18, ratio6_1, false);
motor MotorL2 = motor(PORT17, ratio6_1, true);
motor MotorL3 = motor(PORT16, ratio6_1, true);
encoder EncoderR = encoder(Brain.ThreeWirePort.C);
encoder EncoderL = encoder(Brain.ThreeWirePort.E);
motor flywheel = motor(PORT2, ratio6_1, false);

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