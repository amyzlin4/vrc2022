#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor MotorGroupRMotorA = motor(PORT4, ratio6_1, false);
motor MotorGroupRMotorB = motor(PORT1, ratio6_1, false);
motor_group MotorGroupR = motor_group(MotorGroupRMotorA, MotorGroupRMotorB);
motor MotorGroupLMotorA = motor(PORT3, ratio6_1, true);
motor MotorGroupLMotorB = motor(PORT2, ratio6_1, true);
motor_group MotorGroupL = motor_group(MotorGroupLMotorA, MotorGroupLMotorB);
inertial Inertial = inertial(PORT17);
controller Controller1 = controller(primary);
motor flywheelMotorA = motor(PORT8, ratio6_1, true);
motor flywheelMotorB = motor(PORT9, ratio6_1, false);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);
motor indexer = motor(PORT10, ratio36_1, false);
motor intake = motor(PORT11, ratio18_1, false);

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