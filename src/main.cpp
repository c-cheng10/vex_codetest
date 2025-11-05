#include "main.h"
#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "lemlib/timer.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <string>
#include <cmath>

using namespace Robot;
using namespace Robot::Globals;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button(){
	static bool pressed = false;
	pressed = !pressed;
	if(pressed){
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize(){
    arms.set_value(true);
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    h_encoder.reset_position();
    v_encoder.reset_position();
    chassis.setPose(0,0,0);
}

void screen(){
    // print robot location to the brain screen
    pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // delay to save resources
    pros::delay(20);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled(){}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize(){
    autonomous();
}

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
void opcontrol(){
    int intake_state = 0;
	int score_state = 0;

    bool arms_toggle = false;
    bool matchloader_toggle = false;

    int invert_bool = 1;

	while(true){
        screen();
        // drive control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(invert_bool * leftY, invert_bool * rightX);
		// intake control
		if(controller.get_digital(DIGITAL_L1)){
			intake_state = 1;
		} else if(controller.get_digital(DIGITAL_L2)){
			intake_state = -1;
		} else {
			intake_state = 0;
		}

        // --- Scoring control ---
        if (controller.get_digital(DIGITAL_R1)) {
            score_state = 1;
            intake_state = 1;
        } else if (controller.get_digital(DIGITAL_R2)) {
            score_state = -1;
            intake_state= 1;
        } else{
            score_state = 0;
        }

        // --- Intake motor control ---
        if (intake_state == 1) {
            intake.move_velocity(200);
            container.move_velocity(-200);
        } else if (intake_state == -1) {
            intake.move_velocity(-200);
            container.move_velocity(200);
        } else {
            intake.move_velocity(0);
            container.move_velocity(0);
        }

        // --- Scoring motor control ---
        if (score_state == 1) {
            score.move_velocity(200);
        } else if (score_state == -1) {
            score.move_velocity(-200);
        } else {
            score.move_velocity(0);
        }

        // --- Pneumatics toggles ---
        if (controller.get_digital_new_press(DIGITAL_A)) {
            arms_toggle = !arms_toggle;
            arms.set_value(arms_toggle);
        }

        if(controller.get_digital_new_press(DIGITAL_LEFT)){
            matchloader_toggle = !matchloader_toggle;
            matchloader.set_value(matchloader_toggle);
        }

        if(controller.get_digital_new_press(DIGITAL_RIGHT)){
            invert_bool *= -1;
        }

        pros::delay(20);
    }
}
