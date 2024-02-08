#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"


pros::Motor left_front_motor(15, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor left_middle_motor(20, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor left_back_motor(17, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed

pros::Motor right_front_motor(1, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed
pros::Motor right_middle_motor(18, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed
pros::Motor right_back_motor(3, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed

pros::Motor cata(12, pros::E_MOTOR_GEARSET_36, true);

pros::Motor intake(19, pros::E_MOTOR_GEARSET_06, false);

pros::Imu inertial_sensor(13);


pros::ADIDigitalOut LeftWing('H');
pros::ADIDigitalOut RightWing('G');