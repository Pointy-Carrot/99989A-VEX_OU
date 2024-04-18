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
pros::ADIDigitalOut winch_PTO('E');
pros::ADIDigitalOut wings('G');
pros::ADIDigitalOut hang_release('F');
pros::ADIDigitalOut rear_wing('D');
pros::ADIDigitalOut ball_puncher('H');

// Drivetrain
pros::Motor left_front_motor(1, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_middle_motor(4, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_top_motor(3, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_front_motor(8, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_middle_motor(9, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_top_motor(10, pros::E_MOTOR_GEARSET_06, false);
pros::Motor intake(2, pros::E_MOTOR_GEARSET_18, false);
pros::Motor left_kicker(11, pros::E_MOTOR_GEARSET_36, false);
pros::Motor right_kicker(13, pros::E_MOTOR_GEARSET_36, true);

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
pros::Imu imu(7);
lemlib::OdomSensors sensors {nullptr, nullptr, nullptr, nullptr, &imu};


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

bool flywheel_running;
float setpoint;
float error;
float current_velocity;
float kp; // tune
float ki; // tune
float kd; // tune
float prev_error;
float proportional;
float integral;
float derivative;
float output;
float scaled_output;
float integral_limit = 100; // tune
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



void far_qual(){ // 6 triballs

	chassis.setPose(-6, 62, 90);
	intake = 127;
	pros::delay(300);
	// returning and clearing match load zone
	chassis.follow(qual1_txt, 13, 2250, false);
	chassis.waitUntil(18);
	rear_wing.set_value(true);
	chassis.waitUntilDone();
	rear_wing.set_value(false);
	// pushing in triballs
	chassis.setPose(-56, 38, chassis.getPose().theta);
	chassis.moveToPoint(-54, 48, 1000);
	chassis.turnToHeading(270, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 60});
	chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	intake = -127;
	chassis.waitUntilDone();
	pros::delay(300);
	chassis.setPose(-59.745, 40.512, chassis.getPose().theta);
	pros::delay(150);
	chassis.follow(qual4_txt, 13, 750);
	chassis.waitUntilDone();
	chassis.setPose(-56, 32, chassis.getPose().theta);
	intake = 0;
	// grabbing next triball
	chassis.moveToPoint(-56, 40, 1000, {.forwards = false, .minSpeed = 80});
	chassis.turnToPoint(-4, 36, 500, {});
	chassis.moveToPoint(-24, 36, 2000, {.minSpeed = 110});
	intake = 127;
	chassis.waitUntilDone();
	pros::delay(300);
	// scoring that triball
	chassis.turnToPoint(-44, 4, 750, {});
	chassis.moveToPoint(-44, 10, 1000, {.minSpeed = 120});
	intake = -127;
	chassis.waitUntilDone();
	chassis.setPose(-44, 16, chassis.getPose().theta);
	// grabbing second to last triball (mid)
	chassis.moveToPoint(-20, 20, 1500, {.forwards = false});
	chassis.turnToPoint(0, 0, 500, {});
	chassis.moveToPose(0, 0, 120, 1000);
	intake = 127;
	chassis.waitUntilDone();
	pros::delay(250);
	// scoring last two triballs
	chassis.turnToPoint(-90, 0, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
	chassis.waitUntilDone();
	pros::delay(100);
	wings.set_value(true);
	intake = -127;
	pros::delay(100);
	chassis.moveToPoint(-90, 0, 2000, {.minSpeed = 120});

}




void far_elim(){ // 5 triballs
	
	chassis.setPose(-34, 55, 180);

	// rushing mid
	intake = 127;
	chassis.follow(qual3_txt, 13, 1500);
	chassis.waitUntil(4);
	ball_puncher.set_value(true);
	chassis.waitUntil(58);
	chassis.cancelMotion();
	//returning
	chassis.moveToPoint(-24.5, 56, 2000, {.forwards = false, .minSpeed = 100});
	chassis.waitUntilDone();
	// outtaking triball
	chassis.turnToHeading(240, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
	chassis.waitUntilDone();
	intake = -127;
	pros::delay(400);
	// grabbing elevation bar triball
	chassis.turnToHeading(30, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
	chassis.waitUntilDone();
	chassis.setPose(-30, 56, chassis.getPose().theta);
	intake = 127;
	chassis.moveToPose(-2, 56, 90, 2000, {.earlyExitRange = 2});
	chassis.waitUntilDone();
	chassis.setPose(-5, 62, chassis.getPose().theta);
	// returning and clearing match load zone
	chassis.follow(qual1_txt, 13, 2250, false);
	chassis.waitUntil(18);
	rear_wing.set_value(true);
	chassis.waitUntilDone();
	rear_wing.set_value(false);
	// pushing in triballs
	chassis.setPose(-56, 38, chassis.getPose().theta);
	chassis.moveToPoint(-54, 48, 1000);
	chassis.turnToHeading(270, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 60});
	chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	intake = -127;
	chassis.waitUntilDone();
	chassis.setPose(-59.745, 40.512, chassis.getPose().theta);
	pros::delay(150);
	chassis.moveToPoint(chassis.getPose().x, 0, 750, {.minSpeed = 127});
	chassis.waitUntilDone();
	chassis.setPose(-56, 32, chassis.getPose().theta);
	intake = 0;
	// grabbing next triball
	chassis.moveToPoint(-56, 40, 1000, {.forwards = false, .minSpeed = 80});
	chassis.turnToPoint(-4, 36, 500, {});
	chassis.moveToPoint(-26, 38, 2000, {.minSpeed = 110});
	intake = 127;
	chassis.waitUntilDone();
	pros::delay(300);
	// scoring last two triballs
	chassis.turnToHeading(180, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
	chassis.waitUntilDone();
	// chassis.follow(qual5_2_txt, 13, 2000);
	chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 60});
	wings.set_value(true);
	chassis.moveToPoint(-44, chassis.getPose().y-30, 10000, {.minSpeed = 127});
	intake = -127;
}




void close_elim(){
	chassis.setPose(35, 55, 180);
	// rushing middle triball
	chassis.moveToPose(26, 6, 180, 1500, {.minSpeed = 120});
	intake = 127;
	chassis.waitUntilDone();
	chassis.moveToPoint(36, 46, 2000, {.forwards = false, .minSpeed = 100});
	chassis.turnToHeading(270, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
	chassis.waitUntil(10);
	intake = -127;
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.waitUntilDone();
	chassis.setPose(44, 57.632, chassis.getPose().theta);
	// rushing barrier triball
	chassis.follow(qual6_txt, 18, 2000);
	intake = 127;
	chassis.waitUntilDone();
	chassis.moveToPose(10, 0, 225, 500);
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.follow(qual7_txt, 18, 2000, false);
	// clearing matchload zone
	chassis.waitUntilDone();
	rear_wing.set_value(true);
	pros::delay(300);
	// chassis.moveToPoint(chassis.getPose().x-3, chassis.getPose().y+3, 1000, {.forwards = false});
	// chassis.waitUntilDone();
	chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.waitUntilDone();
	rear_wing.set_value(false);
	pros::delay(300);
	// touching elevation pole
	chassis.turnToHeading(315, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.waitUntilDone();
	chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.waitUntilDone();
	chassis.moveToPoint(7, chassis.getPose().y+4, 2000, {.maxSpeed = 60});
	intake = -127;
	chassis.waitUntilDone();
	pros::delay(100);
	chassis.moveToPoint(38, chassis.getPose().y-6, 2000, {.forwards = false, .minSpeed = 1});
	chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
	intake = 0;
}




void close_qual(){
	chassis.setPose(52, 55, 180);
	intake = 127;
	// clearing match load zone
	rear_wing.set_value(true);
	pros::delay(400);
	chassis.turnToHeading(90, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.waitUntilDone();
	rear_wing.set_value(false);
	// scoring alliance triball
	chassis.turnToHeading(135, 500, {});
	chassis.moveToPoint(chassis.getPose().x+12, chassis.getPose().y-12, 2000);
	chassis.waitUntilDone();
	intake = -127;
	pros::delay(400);
	// touching elevation pole
	chassis.moveToPoint(chassis.getPose().x-20, chassis.getPose().y+26, 2000, {.forwards = false});
	chassis.waitUntilDone();
	chassis.turnToHeading(315, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
	chassis.waitUntilDone();
	chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.moveToPoint(23, chassis.getPose().y, 2000, {.maxSpeed = 40});
}




void skills_macro(){
	chassis.setPose(45, 56, 320);
	chassis.moveToPose(54, 20, 0, 1000, {.forwards = false, .minSpeed = 127});
	chassis.waitUntilDone();
	chassis.setPose(58, 30, chassis.getPose().theta);
	pros::delay(100);
	chassis.turnToPoint(52, 46, 750, {});
	chassis.moveToPoint(50, 48, 1000);
	chassis.waitUntilDone();
	chassis.turnToPoint(0, 32, 750, {});
	chassis.waitUntilDone();
	chassis.moveToPoint(chassis.getPose().x+3, chassis.getPose().y-2, 750, {.forwards = false});
	chassis.turnToPoint(0, 36, 750, {});
	right_kicker = 88;
	left_kicker = 88;
}




void prog_skills(){ // programming skills
	skills_macro();
	pros::delay(28000);
	chassis.turnToPoint(44.452, 56, 750, {});
	chassis.moveToPose(30, 56, 270, 750);
	chassis.moveToPose(-46, 56, 270, 2000, {.minSpeed = 110});
	right_kicker = 0;
	left_kicker = 0;
	chassis.moveToPose(-60, 20, 180, 2000, {.minSpeed = 120});
	chassis.waitUntilDone();
	chassis.setPose(-58, 28, chassis.getPose().theta);

	chassis.moveToPose(-46, 54, 45, 1000, {.forwards = false, .minSpeed = 90});
	chassis.waitUntilDone();
	chassis.turnToPoint(-68, 46, 500, {.forwards = false});
	chassis.moveToPose(-68, 10, 0, 2000, {.forwards = false, .minSpeed = 120});
	chassis.waitUntilDone();
	chassis.setPose(-58, 30, chassis.getPose().theta);
	chassis.moveToPose(-46, 54, 45, 1000, {.minSpeed = 90});
	chassis.turnToPoint(-68, 46, 750, {.forwards = false});
	chassis.moveToPose(-68, 10, 0, 2000, {.forwards = false, .minSpeed = 120});
	chassis.waitUntilDone();
	chassis.setPose(-58, 30, chassis.getPose().theta);
	chassis.moveToPose(-46, 54, 45, 1000, {.minSpeed = 90});
	chassis.turnToPoint(-68, 46, 750, {.forwards = false});
	chassis.moveToPose(-68, 10, 0, 2000, {.forwards = false, .minSpeed = 120});
	chassis.waitUntilDone();
	chassis.setPose(-58, 30, chassis.getPose().theta);

	chassis.moveToPoint(-56, 34, 1000);
	chassis.turnToPoint(0, 34, 750, {});
	chassis.waitUntilDone();
	wings.set_value(true);
	chassis.follow(Skills3_txt, 13, 1250);
	chassis.waitUntil(16);
	wings.set_value(false);
	chassis.waitUntilDone();
	chassis.turnToPoint(-11.345, -90, 750, {});
	chassis.waitUntilDone();
	wings.set_value(true);
	pros::delay(500);
	chassis.moveToPose(-50, 12, 270, 2000, {.minSpeed = 120});

	chassis.waitUntilDone();
	wings.set_value(false);
	chassis.setPose(-44, 16, chassis.getPose().theta);
	chassis.moveToPoint(-10, 16, 1000, {.forwards = false});
	chassis.waitUntilDone();
	wings.set_value(true);
	chassis.moveToPose(-50, 16, 270, 1250, {.minSpeed = 120});
	chassis.waitUntilDone();
	chassis.setPose(-44, 16, chassis.getPose().theta);
	wings.set_value(false);

	chassis.moveToPoint(-10, 16, 1500, {.forwards = false});
	chassis.turnToPoint(-6, -90, 750, {});
	chassis.moveToPoint(-6, -30, 1250);
	chassis.turnToPoint(-90, -30, 750, {});
	chassis.moveToPoint(-10, -30, 1000);
	chassis.turnToPoint(-20, 90, 750, {});
	chassis.waitUntilDone();
	wings.set_value(true);
	chassis.moveToPose(-50, -4, 270, 1500, {.minSpeed = 120});
	chassis.waitUntilDone();
	chassis.setPose(-44, -2, chassis.getPose().theta);
	wings.set_value(false);
	chassis.moveToPoint(0, -2, 500, {.forwards = false});
}

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
			close_elim();
			break;
		case -3:
			far_elim();
			break;
		case -2:
			close_qual();
			break;
		case -1:
			far_qual();
			break;
		case 0:
			prog_skills();
			break;
		case 1:
			far_elim();
			break;
		case 2:
			close_qual();
			break;
		case 3:
			far_elim();
			break;
		case 4:
			close_elim();
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

	bool hang_released = false;
	bool PTO_activated = false;
	bool rear_wing_activated = false;

	wings.set_value(false);
	
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

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake = 127;
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake = -127;
		} else{
			intake = 0;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			if(!hang_released){
				hang_release.set_value(true);
				hang_released = true;
			} else{
				hang_release.set_value(false);
				hang_released = false;
			}
		} else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			if(!PTO_activated){
				winch_PTO.set_value(true);
				PTO_activated = true;
			} else{
				winch_PTO.set_value(false);
				PTO_activated = false;
			}
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			wings.set_value(true);
		} else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			wings.set_value(false);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			right_kicker = 88;
			left_kicker = 88;
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			right_kicker.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			left_kicker.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			right_kicker = 0;
			left_kicker = 0;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			skills_macro();
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			if(!rear_wing_activated){
				rear_wing.set_value(true);
				rear_wing_activated = true;
			} else{
				rear_wing.set_value(false);
				rear_wing_activated = false;
			}
		}
		pros::delay(20);
	}
}
