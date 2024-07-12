#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"


#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <string>


pros::MotorGroup left_drivetrain({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup right_drivetrain({right_front_motor, right_middle_motor, right_back_motor});


lemlib::Drivetrain_t drivetrain {
    &left_drivetrain, // left drivetrain motors
    &right_drivetrain, // right drivetrain motors
    12.75, // track width
    3.25, // wheel diameter
    450 // wheel rpm
};

lemlib::OdomSensors_t sensors {
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &inertial_sensor // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
	15, // kP
	// 10,
	30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    5000, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    10, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};



lemlib::Chassis chassis(drivetrain, lateralController, angularController,sensors);

/**	
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


//ODOM SCREEN
void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "25600Y - Free Bird");
	pros::Task screenTask(screen);


	chassis.calibrate(); // calibrate the chassis
	chassis.setPose(0,0,0);
    // chassis.setPose(33.25, -26, 158.92); // X: 0, Y: 0, Heading: 0
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	intake.move(127);
	
	chassis.moveTo(-20, -21, 2000, 70);

	intake.move(0);
	
	chassis.turnTo(-1000, -21, 1000,100);

	chassis.moveTo(-50, -21, 1000,80);

	chassis.moveTo(-19,-20, 1000, 100);

	chassis.turnTo(10000, -21, 1500, 60);
	
	LeftWingBack.set_value(true);

	chassis.moveTo(2,-15,2000,150);

	LeftWingBack.set_value(false);

	chassis.turnTo(0,1000,2000,40);

	chassis.moveTo(0,22,1000,80);

}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	bool LeftWingFrontValue = false;
	bool RightWingFrontValue = false;
	bool LeftWingBackValue = false;
	bool RightWingBackValue = false;
	bool CataToggle = false;
	while (true) {

		//DRIVING CONTROLS

		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		

		float LeftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		float RightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// std::cout<<LeftY<<std::endl;
		// std::cout<<RightY<<std::endl;

		left_drivetrain.move(LeftY);
		right_drivetrain.move(RightY);

		//INTAKE AND CATA CONTROLS

		//INTAKE
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake.move_voltage(12000);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move_voltage(-12000);
		}
		else{
			intake.move_voltage(0);
		}
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
			CataToggle = !CataToggle;
		}
		
		//CATA
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || CataToggle){
			cata.move_velocity(70);
		}
		else{
			cata.move_velocity(0);
		}
		//WINGS
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			LeftWingFrontValue = !LeftWingFrontValue;
			LeftWingFront.set_value(!LeftWingFrontValue);
		} 

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			LeftWingBackValue = !LeftWingBackValue;
			LeftWingBack.set_value(!LeftWingBackValue);
		} 

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			RightWingFrontValue = !RightWingFrontValue;
			RightWingFront.set_value(!RightWingFrontValue);
		} 

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			RightWingBackValue = !RightWingBackValue;
			RightWingBack.set_value(!RightWingBackValue);
		}
		pros::delay(100);
	}
}
