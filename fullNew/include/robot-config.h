using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor intake;
extern controller Controller1;
extern digital_out roller;
extern digital_out expand;
extern inertial Inertial;
extern motor MotorR1;
extern motor MotorR2;
extern motor MotorR3;
extern motor MotorL1;
extern motor MotorL2;
extern motor MotorL3;
extern encoder EncoderR;
extern encoder EncoderL;
extern motor flywheel;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );