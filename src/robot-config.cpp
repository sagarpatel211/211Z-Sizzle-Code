#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RightBackDrive = motor(PORT17, ratio18_1, true);
motor RightFrontDrive = motor(PORT18, ratio18_1, true);
motor LeftBackDrive = motor(PORT14, ratio18_1, false);
motor LeftFrontDrive = motor(PORT20, ratio18_1, false);
motor TrayTilter = motor(PORT15, ratio36_1, false);
motor IntakeArm = motor(PORT1, ratio36_1, true);
motor RightIntake = motor(PORT13, ratio18_1, false);
motor LeftIntake = motor(PORT10, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}