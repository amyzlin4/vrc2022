using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group driveR;
extern motor_group driveL;
extern motor_group flywheel;
extern motor intake;
extern motor indexer;
extern controller Controller1;
extern digital_out roller;
extern digital_out expand;
extern inertial Inertial;
extern distance Distance;
extern gps GPS;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );