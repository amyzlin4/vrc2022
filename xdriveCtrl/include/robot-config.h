using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern motor Motor1FL;
extern motor Motor2FR;
extern motor Motor3BL;
extern motor Motor4BR;
extern controller Controller1;
extern inertial Inertial15;
extern encoder EncoderR;
extern encoder EncoderL;
extern signature Vision7__DUCK;
extern signature Vision7__R;
extern signature Vision7__B;
extern signature Vision7__SIG_4;
extern signature Vision7__SIG_5;
extern signature Vision7__SIG_6;
extern signature Vision7__SIG_7;
extern vision Vision7;
extern distance DistanceS;
extern optical OpticalS;
extern rotation RotationS;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );