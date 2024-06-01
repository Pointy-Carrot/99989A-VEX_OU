#include "main.h"
#include "display/lv_core/lv_obj.h"
#include "fmt/format.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"
#include "sylib/system.hpp"
#include "gif-pros/gifclass.hpp"
#include "assset.h"
#include "autoSelect/selection.h"
#include <atomic>
#include <cstdint>


ASSSET(taco_gif);
ASSET(Skills1_txt);
ASSET(Skills2_txt);
ASSET(Skills3_txt);
ASSET(qual1_txt);
ASSET(qual1_2_txt);
ASSET(qual2_txt);
ASSET(qual3_txt);
ASSET(qual4_txt);
ASSET(qual4_2_txt);
ASSET(qual5_txt);
ASSET(qual5_2_txt);
ASSET(qual6_txt);
ASSET(qual7_txt);

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

// LED Setup
// auto left_underglow = sylib::Addrled(22, 6, 21);
// auto right_underglow = sylib::Addrled(22, 8, 21);

// Motor Setup

// Pneumatics
pros::ADIDigitalOut mogo_clamp('H');

// Drivetrain
pros::Motor left_front_motor(1, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_middle_motor(12, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_top_motor(11, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_front_motor(10, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_middle_motor(19, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_top_motor(20, pros::E_MOTOR_GEARSET_06, true);

pros::Motor lift_motor(4, pros::E_MOTOR_GEARSET_36);

pros::MotorGroup left_side_motors({left_front_motor, left_middle_motor, left_top_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_middle_motor, right_top_motor});

lemlib::Drivetrain drivetrain {
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
	//wheel base is 8"
    10.5, // track width
    3.25, // wheel diameter
    450, // wheel rpm
	8 // chase power
};


// Sensors
pros::Imu imu(6);
// horizontal tracking wheel encoder
pros::ADIEncoder horizontal_encoder('E', 'F', false);
// vertical tracking wheel encoder
pros::ADIEncoder vertical_encoder('A', 'B', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -8.5);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 1.25);


lemlib::OdomSensors sensors {&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu};


// PIDs

// forward/backward PID
lemlib::ControllerSettings linearController {
    96, // kP
	0, // kI
    0, // kD
	3, // Anti Windup
    1.5, // smallErrorRange
    100, // smallErrorTimeout
    3.5, // largeErrorRange
    500, // largeErrorTimeout
    127 // slew rate
};

// turning PID
lemlib::ControllerSettings angularController {
    4, // kP
	0, // kI
    28, // kD
	3, // Anti Windup
    1.5, // smallErrorRange
    100, // smallErrorTimeout
    3.5, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};


// Chassis

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

// Driver Control Functions


bool lift_up = false;
bool lift_down = true;
bool lift_running = false;



void set_brake(){
	left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_middle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_top_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
void set_hold(){
	left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_middle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_top_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void set_coast(){
	left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_middle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_top_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);		
}


// Screen View

void screen() {
    // loop forever
    /*while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }*/
}

bool auton_started = false;
int auto_num = 0;


float ypos;
float xpos;
float thetapos;
// AUTONS GO HERE




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	sylib::initialize();
	chassis.calibrate();
	selector::init();
	// LED Loop
	// left_underglow.gradient(0xFF0000, 0x0000FF, 0, 0, false, true);
	// left_underglow.cycle(*left_underglow, 5);
	// right_underglow.gradient(0xFF0000, 0x0000FF, 0, 0, false, true);
	// right_underglow.cycle(*right_underglow, 5);
	std::uint32_t clock = sylib::millis();
	pros::Task task([&clock](){
		sylib::delay_until(&clock, 10);
	});
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
	auton_started = true;
	switch(selector::auton){
		case -4:
			break;
		case -3:
			break;
		case -2:
			break;
		case -1:
			break;
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
	}
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
	
	lv_obj_t* obj = lv_obj_create(lv_scr_act(), NULL);
	lv_obj_set_size(obj, 480, 240);
	lv_obj_set_style(obj, &lv_style_transp); // make the container invisible
	lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 60, 0);

	Gif gif(taco_gif, obj);


	set_coast();


	
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);


		// Controls
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightX = master.get_analog(ANALOG_RIGHT_X);

		chassis.curvature(leftY, rightX, 0);

		// Chassis Controls

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			lift_motor = 127;
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			lift_motor = -127;
		} else{
			lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lift_motor = 0;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			mogo_clamp.set_value(true);
		} else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			mogo_clamp.set_value(false);
		}
		pros::delay(20);
	}
}
