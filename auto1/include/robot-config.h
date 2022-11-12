using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group MotorGroupR;
extern motor_group MotorGroupL;
extern inertial Inertial;
extern controller Controller1;
extern motor_group flywheel;
extern motor indexer;
extern motor intake;
extern distance Distance;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );