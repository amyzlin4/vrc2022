#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor driveRMotorA = motor(PORT12, ratio6_1, false);
motor driveRMotorB = motor(PORT11, ratio6_1, false);
motor_group driveR = motor_group(driveRMotorA, driveRMotorB);
motor driveLMotorA = motor(PORT14, ratio6_1, true);
motor driveLMotorB = motor(PORT13, ratio6_1, true);
motor_group driveL = motor_group(driveLMotorA, driveLMotorB);
motor flywheelMotorA = motor(PORT4, ratio6_1, true);
motor flywheelMotorB = motor(PORT6, ratio6_1, false);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);
motor intake = motor(PORT3, ratio18_1, true);
motor indexer = motor(PORT1, ratio18_1, false);
controller Controller1 = controller(primary);

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