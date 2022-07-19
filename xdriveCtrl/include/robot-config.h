using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor Motor1FL;
extern motor Motor2FR;
extern motor Motor3BL;
extern motor Motor4BR;
extern controller Controller1;
extern inertial Inertial15;
extern encoder EncoderR;
extern encoder EncoderL;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );