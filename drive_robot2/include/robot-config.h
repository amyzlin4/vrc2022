using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group driveR;
extern motor_group driveL;
extern motor_group flywheel;
extern motor intake;
extern motor indexer;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );