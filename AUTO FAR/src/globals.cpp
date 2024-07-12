#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"


pros::Motor left_front_motor(10, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor left_middle_motor(18, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor left_back_motor(17, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed

pros::Motor right_front_motor(11, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed
pros::Motor right_middle_motor(12, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed
pros::Motor right_back_motor(16, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed

pros::Motor cata(20, pros::E_MOTOR_GEARSET_36, true);

pros::Motor intake(13, pros::E_MOTOR_GEARSET_06, false);

pros::Imu inertial_sensor(3);


pros::ADIDigitalOut LeftWingBack('A');
pros::ADIDigitalOut RightWingBack('B');

pros::ADIDigitalOut LeftWingFront('H');
pros::ADIDigitalOut RightWingFront('G');