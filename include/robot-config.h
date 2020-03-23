using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RightBackDrive;
extern motor RightFrontDrive;
extern motor LeftBackDrive;
extern motor LeftFrontDrive;
extern motor TrayTilter;
extern motor IntakeArm;
extern motor RightIntake;
extern motor LeftIntake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );