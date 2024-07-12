#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller = controller(primary);
motor LeftDriveFront = motor(PORT15, ratio6_1, true);
motor LeftDriveMiddle = motor(PORT20, ratio6_1, true);
motor LeftDriveBack = motor(PORT17, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(LeftDriveFront, LeftDriveMiddle, LeftDriveBack);

motor RightDriveFront = motor(PORT1, ratio6_1, false);
motor RightDriveMiddle = motor(PORT18, ratio6_1, false);
motor RightDriveBack = motor(PORT3, ratio6_1, false);
motor_group RightDriveSmart = motor_group(RightDriveFront, RightDriveMiddle, RightDriveBack);

digital_out LeftWing = digital_out(Brain.ThreeWirePort.H);
digital_out RightWing = digital_out(Brain.ThreeWirePort.G);

// digital_out Expansion2 = digital_out(Brain.ThreeWirePort.F);

  //if no inertial, use this:
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);


  //with an inertial:
// smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 165, mm, 1);
//inertial TurnGyroSmart = inertial(PORT14);

motor Intake = motor(PORT19, ratio6_1, false);

motor Cata = motor(PORT12, ratio36_1, false);

bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}