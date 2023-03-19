using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor intake;
extern controller Controller1;
extern inertial Inertial;
extern motor MotorR1;
extern motor MotorR2;
extern motor MotorR3;
extern motor flywheel;
extern digital_out angler;
extern digital_out expand;
extern motor MotorL1;
extern motor MotorL2;
extern motor MotorL3;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );