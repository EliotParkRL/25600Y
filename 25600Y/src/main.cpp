/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
using namespace vex;
#include <cmath>
#include <string>
#include <iostream>
#include <stdio.h>

#include "robot-config.h"
// A global instance of competition
competition Competition;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void sleep(int time){
  vex::task::sleep(time);
}

int deadzone(int num){
   return 0.5* num*pow(abs(num), 0.75)/(pow(127, 0.75));

 }

static double deadzoneVariable = 10;

int leftDeadzone(){
  if (abs(Controller.Axis3.value()) < deadzoneVariable ){
    return 0;
  }
  else{
    return 1;
  }
}

int rightDeadzone(){
  if (abs(Controller.Axis2.value()) < deadzoneVariable ){
    return 0;
  }
  else{
    return 1;
  }
}

int xp;
void touchPos(){
    xp = Brain.Screen.xPosition();
}    





void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //Blooper.set(false);
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AUTONOMOUS FUNCTIONS                             */
/*                                                                           */
/*---------------------------------------------------------------------------*/

//Settings:
double speedDiv = 1;
double turnSpeedDiv = 1;

double newSpeedDiv;
double newTurnSpeedDiv;

double kP = 0.101;
double kI = 0.0004;
double kD = 0.07;

double turnkP = 0.101; //was 0.12
double turnkI = 0.0004;
double turnkD = 0.04;

//Autonomous Settings
int setPoint = 200;
int turnSetPoint = 0;

int error;  // Sensor Value - Desired Value : Positional Value (it's like delta x)
int prevError = 0; // Position 20msec ago
int derivative; //error - prevError : Speed
int integral = 0; //  integral =  integral + error   : Position -> Absement (integral stuff)
int nullifiedIntegral = 0;

int turnError;  // Sensor Value - Desired Value : Positional Value (it's like delta x)
int turnPrevError = 0; // Position 20msec ago
int turnDerivative; //error - prevError : Speed
int turnIntegral = 0; //  integral =  integral + error   : Position -> Absement (integral stuff)

int maxIntegral = 300;
int integralBound = 20;
int turnMaxIntegral = 300;


double signnum_c(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}

bool resetDriveSensors = false;
bool enableDrivePID = true;
bool useSpeedDiv = false;

int drivePID() {
  while(enableDrivePID == true) {
    speedDiv = 1;
    turnSpeedDiv = 1;

    if(resetDriveSensors) {
      resetDriveSensors = false;

      //LeftDriveSmart.setPosition(0, degrees);
      LeftDriveBack.setPosition(0, degrees);
      //RightDriveSmart.setPosition(0, degrees);
      RightDriveBack.setPosition(0, degrees);
    }

    if(useSpeedDiv) {
      useSpeedDiv = false;
      speedDiv = newSpeedDiv;
      turnSpeedDiv = newTurnSpeedDiv;
    }

    int leftMotorPos = LeftDriveBack.position(degrees);
    int rightMotorPos = RightDriveBack.position(degrees);

    /*----------------------*/
    /* Lateral Movement PID */
    /*----------------------*/

    int averagePos = (leftMotorPos + rightMotorPos) / 2;
    int sensorValue = averagePos;

    //Potential
    error = setPoint - sensorValue; //negative

    //Derivative
    derivative = error - prevError;

    //Integral
    //integral += error;

    if(abs(error) < integralBound){
      integral += error; 
    }  else {
      integral = 0; 
    }

    integral = abs(integral) > maxIntegral ? signnum_c(integral) * maxIntegral : integral;

    double lateralMotorPower = (error * kP + derivative * kD +  integral * kI) * 0.3; //voltage is 12.0

    /*----------------------*/
    /* Turning Movement PID */
    /*----------------------*/

    int turnDifference = leftMotorPos - rightMotorPos;

    //Potential
    turnError = turnSetPoint - turnDifference;

    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral
    //turnIntegral += turnError;

    if(abs(turnError) < integralBound){
      turnIntegral += turnError; 
    }  else {
      turnIntegral = 0; 
    }

    turnIntegral = abs(turnIntegral) > maxIntegral ? signnum_c(turnIntegral) * maxIntegral : turnIntegral;

    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD + turnIntegral * turnkI) * 0.3; //voltage is 12.0

    LeftDriveBack.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LeftDriveMiddle.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LeftDriveFront.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);

    RightDriveBack.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    RightDriveMiddle.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    RightDriveFront.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    //code
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(15);

    std::cout << Brain.Timer.value() << " , " 
    << LeftDriveBack.velocity(rpm) << " , " <<  LeftDriveBack.torque(Nm) << " , " 
    << LeftDriveBack.current() << " , " << LeftDriveBack.voltage(volt)
    << std::endl; 
  }

  return 1;
}


void drive_till_stop(int speed, double rot){
  // int leftSpeed = static_cast<int>(speed * 0.7);
  // LeftDriveFront.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveMiddle.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveBack.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false); //was false
  // RightDriveFront.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveMiddle.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveBack.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);

  LeftDriveSmart.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, true);
  RightDriveSmart.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, true);
}

void drive_no_stop(int speed, double rot){
  // LeftDriveFront.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveMiddle.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveBack.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false); 
  // RightDriveFront.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveMiddle.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveBack.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);

  LeftDriveSmart.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  RightDriveSmart.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);

}  

void drive_back_no_stop(int speed, double rot){
  // LeftDriveFront.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveMiddle.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveBack.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false); 
  // RightDriveFront.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveMiddle.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveBack.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);

  LeftDriveSmart.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  RightDriveSmart.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
}

void drive_back_till_stop(int speed, double rot){
  // LeftDriveFront.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveMiddle.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveBack.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false); 
  // RightDriveFront.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveMiddle.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveBack.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, true);

  LeftDriveSmart.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, true);
  RightDriveSmart.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, true);
}

void turn_till_stop(int speed, double rot){
  // LeftDriveFront.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveMiddle.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // LeftDriveBack.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveFront.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveMiddle.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  // RightDriveBack.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);

  LeftDriveSmart.spinFor(vex::directionType::rev, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, true);
  RightDriveSmart.spinFor(vex::directionType::fwd, rot, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, true);
}



double speed_calc(double revs, double dist, double accel_dist, double decel_dist, int maximum_speed){
  //revs: how many revolutions you've already done
  //dist: how many revolutions left to go
  
  double speed;
  double max_speed = -maximum_speed; ///THIS IS THE MAX SPEED
  double init_speed = -20;

  if(revs >= -accel_dist){
    speed = init_speed - (max_speed - init_speed) * (1 / accel_dist) * (revs);
    //go from -20 to -150 in 1 revolution
  }
  else if(dist >= -decel_dist){
    speed =  max_speed - (max_speed * 1 / decel_dist) * (decel_dist + dist) - 10;
  }

  else{
    speed = max_speed;
  }
  return speed;
}


double speed_calc_back(double revs, double dist, double accel_dist, double decel_dist, int maximum_speed){
  //revs: how many revolutions you've already done
  //dist: how many revolutions left to go
  
  double speed;
  double max_speed = maximum_speed; ///THIS IS THE MAX SPEED
  double init_speed = 20;

  if(revs <= accel_dist){
    speed = init_speed + (max_speed - init_speed) * (1 / accel_dist) * (revs);
    //go from -20 to -150 in 1 revolution
  }
  else if(dist <= decel_dist){
    speed =  max_speed - (max_speed * 1 / decel_dist) * (decel_dist-dist) + 10;
  }

  else{
    speed = max_speed;
  }

  return speed;
}

void gradient_drive(double revs, double accel_dist, double decel_dist, int maximum_speed){
  RightDriveFront.setPosition(0, rev);
  RightDriveMiddle.setPosition(0, rev);
  RightDriveBack.setPosition(0, rev);
  LeftDriveFront.setPosition(0, rev);
  LeftDriveMiddle.setPosition(0, rev);
  LeftDriveBack.setPosition(0, rev);
  revs = revs*(-1);

  wait(0.15,sec); //was 0.3
    Brain.Screen.print("Reading");
    Brain.Screen.print(RightDriveFront.position(rev));
    while(RightDriveFront.position(rev)> revs){
      double reading = RightDriveFront.position(rev);
      double speed = speed_calc(reading, revs-reading, accel_dist, decel_dist, maximum_speed);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print(" ");
      Brain.Screen.print(revs-reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speed);
      Brain.Screen.newLine();

      //std::cout << reading << std::endl;
      //std::cout << revs - reading << std::endl;
      
      std::cout << "Distance" << std::endl;
      std::cout << revs-reading << std::endl;      

      std::cout << "Speed" << std::endl;
      std::cout << speed <<std::endl;

    
      Drivetrain.drive(fwd, speed, rpm);

      wait(1,msec);

    }

    Drivetrain.stop(hold);

}



void gradient_drive_back(double revs, double accel_dist, double decel_dist, int maximum_speed){
  RightDriveFront.setPosition(0, rev);
  RightDriveMiddle.setPosition(0,rev);
  RightDriveBack.setPosition(0, rev);
  LeftDriveFront.setPosition(0, rev);
  LeftDriveMiddle.setPosition(0, rev);
  LeftDriveBack.setPosition(0, rev);

  wait(0.15,sec); //was 0.3

    while(RightDriveFront.position(rev) < revs){
      double reading = RightDriveFront.position(rev);
      double speed = speed_calc_back(reading, revs-reading, accel_dist, decel_dist, maximum_speed);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print(" ");
      Brain.Screen.print(revs-reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speed);
      Brain.Screen.newLine();


      Drivetrain.drive(fwd, speed, rpm);

      wait(1,msec);

    }

    Drivetrain.stop(hold);

}

/*---------------------------------------------------------------------------*/
/*                          Inertial Sensor Required                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/



void make_turn(double speed, double ang, double tresh){

  while(Drivetrain.heading() < ang-tresh || Drivetrain.heading() > ang + tresh){

    Drivetrain.turn(right,speed, rpm);

  }
  Drivetrain.stop();


}

void auton_turn(double speed, double ang){
  make_turn(speed, ang, 10);
  std::cout << "Angle" << std::endl;
  std::cout << Drivetrain.heading() <<std::endl;
  wait(200,msec);
  std::cout << Drivetrain.heading() <<std::endl;
  
  if(Drivetrain.heading() > ang-0.5){ // > 0.5
    std::cout << "Correcting A" << std::endl;
    make_turn(9,ang,0.5);
    std::cout << Drivetrain.heading() <<std::endl;
  }

  else if(Drivetrain.heading() < ang+0.5){
    std::cout << "Correcting B" << std::endl;
    make_turn(-9,ang,1.0);
  }

}

// /////////////////////////////////////////////////////////////////////////////////////////////////////
// //Pid guy
// void PIDDrive(int desiredLateral, int desiredTurn) {
//   resetDriveSensors = true;
//   setPoint = desiredLateral;
//   turnSetPoint = desiredTurn;
//   vex::task::sleep(100);
// }

// void PIDDrive(int desiredLateral, int desiredTurn, double speed, double turnSpeed) {
//   newSpeedDiv = speed;
//   newTurnSpeedDiv = turnSpeed;
//   resetDriveSensors = true;
//   setPoint = desiredLateral;
//   turnSetPoint = desiredTurn;
//   vex::task::sleep(100);
// }

// void autonShootDisc(int numShots, double currentFlywheelValue) {
//   for(int i = 0; i < numShots; i++) {
//     Flywheel.spin(fwd, currentFlywheelValue + 0.5, voltageUnits::volt);
//     Intake.spin(reverse, 12, voltageUnits::volt);
//     wait(250, msec);
//     Intake.stop();
//     wait(950, msec);
//   }

// }



/////////////////////////////////////////////////////////////////////////////////////////////////////



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//auton


  // LeftDriveSmart.drive_till_stop(100, 2);
  // drive_till_stop(100,2);



// SET isSkills TO FALSE IF NOT RUNNING SKILLS


// bool offRoller = false;
// bool onRoller = true;  
bool isSkills = true;

// bool done = false;i
void autonomous(void) {
  TurnGyroSmart.resetHeading();
  TurnGyroSmart.resetRotation();
  LeftWing.set(false);
  RightWing.set(false);

  // Cata.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
  // wait(29,sec);

  // Cata.stop();


  auton_turn(80,90);
  
  // drive_till_stop(100,4);
  // wait(0.7,sec);
  // drive_back_till_stop(100,0.8);
  // wait(0.5,sec);              //MATCH AUTON
  // drive_till_stop(100,4);
  // wait(1,sec);




  
  

}
// ////////////      NUMBER ONE
// /////////////////////////////////////////////////////////////////////////////////////////////////////////
// ///On Roller
// /////////////////////////////////////////////////////////////////////////////////////////////////////////
//   if(onRoller) {

//     /////     Spin Roller
//     Flywheel.spin(fwd, 11.7, voltageUnits::volt);
//     Intake.spin(fwd, 12, voltageUnits::volt);
//     PIDDrive(-50, 0);
//     wait(100, msec);

//     /////     Pick up first disc
//     PIDDrive(200, 0);
//     wait(400, msec);
//     PIDDrive(0, -420);
//     wait(500, msec);
//     PIDDrive(380, 0);
//     wait(500, msec);

//     /////     Turn and shoot 3 discs
//     PIDDrive(-180, 0); //was 410
//     wait(300, msec);
//     PIDDrive(0, 365);
//     wait(510, msec);
//     Intake.stop();
//     autonShootDisc(3, 11.55);
//     Flywheel.stop();

//     /////     Back up to align with 3-line
//     PIDDrive(-165, 0);
//     wait(200, msec);
//     PIDDrive(0, 630);
//     wait(800, msec);
//     Intake.spin(fwd, 12.0, voltageUnits::volt);
//     Flywheel.spin(fwd, 11.3, voltageUnits::volt); //pre-revv
//     PIDDrive(2000, 0);
//     wait(1800, msec);
//     PIDDrive(900, 0);
//     wait(1000, msec);

//     /////     Align and shoot 3 discs
//     PIDDrive(0, -670);
//     wait(1000, msec);
//     Intake.stop();
//     autonShootDisc(3, 11.2);
//     Flywheel.stop();
//}
// ////////////      NUMBER TWO
// /////////////////////////////////////////////////////////////////////////////////////////////////////////
// ///Off Roller
// /////////////////////////////////////////////////////////////////////////////////////////////////////////
//   if(offRoller) {
//     PIDDrive(-1170, 0);
//     wait(2, sec);
//     PIDDrive(0, 900);
//     wait(1, sec);
//     Intake.spin(fwd, 12, voltageUnits::volt);
//     PIDDrive(-530, 0);
//     wait(900, msec);
//     Intake.stop();
//   }

// }



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {
  LeftWing.set(false);
  RightWing.set(false);



  enableDrivePID = false;
  // User control code here, inside the loop
  Brain.Screen.clearScreen(vex::color::black);
  Brain.Screen.drawRectangle(0,0,240,240,vex::color::blue);
  Brain.Screen.drawRectangle(240,0,240,240,vex::color::red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawLine(0, 120, 480, 120);
  Brain.Screen.printAt(100,180,"LEFT");
  Brain.Screen.printAt(100,60,"RIGHT");
  Brain.Screen.printAt(340,180,"LEFT");
  Brain.Screen.printAt(340,60,"RIGHT");
  Brain.Screen.render();
  while (1) {

  wait(20, msec); //THIS WAS 5 MSEC
  

  
  
  if(Brain.Screen.pressing()){
    if (Brain.Screen.xPosition()<240  && Brain.Screen.yPosition() > 120){
      // offRoller = true;
      // onRoller = false;
      Brain.Screen.drawRectangle(0,0,480,240,vex::color::blue);
      Brain.Screen.printAt(200,120,"OFF ROLLER");
    }
    else if (Brain.Screen.xPosition()<240 && Brain.Screen.yPosition() < 120) {
      // offRoller = false;
      // onRoller = true;

      Brain.Screen.drawRectangle(0,0,480,240,vex::color::blue);
      Brain.Screen.printAt(200,120,"ON ROLLER");
    }
    else if (Brain.Screen.xPosition()>240 && Brain.Screen.yPosition() > 120) {

      // offRoller = false;
      // onRoller = false;
      Brain.Screen.drawRectangle(0,0,480,240,vex::color::red);
      Brain.Screen.printAt(200,120,"OFF ROLLER STEAL");
    }
    else if (Brain.Screen.xPosition()>240 && Brain.Screen.yPosition() < 120) {
      // offRoller = false;
      // onRoller = false;
      Brain.Screen.drawRectangle(0,0,480,240,vex::color::red);
      Brain.Screen.printAt(200,120,"ON ROLLER ELBOW");
    }
    Brain.Screen.render();
    
  }
  
  // This is the main execution loop for the user control program.
  // Each time through the loop your program should update motor + servo
  // values based on feedback from the joysticks.

  wait(5, msec); // Sleep the task for a short amount of time to
  
  
  
  //LeftDrive.spin(vex::directionType::fwd, 0.8*deadzone(Controller.Axis3.value()), vex::velocityUnits::pct);
  //RightDrive.spin(vex::directionType::fwd, 0.8*deadzone(Controller.Axis2.value()), vex::velocityUnits::pct);

    LeftDriveFront.spin(vex::directionType::fwd, 12*deadzone(Controller.Axis3.value()), vex::velocityUnits::pct);
    LeftDriveMiddle.spin(vex::directionType::fwd, 12*deadzone(Controller.Axis3.value()), vex::velocityUnits::pct);
    LeftDriveBack.spin(vex::directionType::fwd, 12*deadzone(Controller.Axis3.value()), vex::velocityUnits::pct);
    //was -12 deadzone
    RightDriveFront.spin(vex::directionType::fwd, 12*deadzone(Controller.Axis2.value()), vex::velocityUnits::pct);
    RightDriveMiddle.spin(vex::directionType::fwd, 12*deadzone(Controller.Axis2.value()), vex::velocityUnits::pct);
    RightDriveBack.spin(vex::directionType::fwd, 12*deadzone(Controller.Axis2.value()), vex::velocityUnits::pct);

  if(Controller.ButtonDown.pressing()){
    LeftWing.set(true);
  }
  if(Controller.ButtonUp.pressing()){
    LeftWing.set(false);
  }
   if(Controller.ButtonX.pressing()){
    RightWing.set(false);
  }
   if(Controller.ButtonB.pressing()){
    RightWing.set(true);
  }


  if(Controller.ButtonL2.pressing()){
    Cata.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    //Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);  //was fwd, -100
  }else {
    Cata.stop();
    //Intake.stop();
  }

  if(Controller.ButtonR1.pressing()){
    //Flywheel.spin(vex::directionType::fwd, 10.3, vex::voltageUnits::volt);
    Intake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);

  } 
  else if(Controller.ButtonR2.pressing()){
    Intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);

  } 
  
  else {
    Intake.stop();
  }

  }

}
//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }

}  
