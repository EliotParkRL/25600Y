using namespace vex;

extern brain Brain;

extern digital_out LeftWing;
extern digital_out RightWing;

// VEXcode devices
extern controller Controller;
extern drivetrain Drivetrain;
// extern smartdrive Drivetrain;

extern motor Intake;
extern motor Cata;
//extern inertial TurnGyroSmart;

extern motor LeftDriveFront;
extern motor LeftDriveMiddle;
extern motor LeftDriveBack;
extern motor_group LeftDriveSmart;

extern motor RightDriveFront;
extern motor RightDriveMiddle;
extern motor RightDriveBack;
extern motor_group RightDriveSmart;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );