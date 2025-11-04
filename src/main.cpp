/*
  #    ###    #    ###    ###   #### 
 ##   #   #  ##   #   #  #   #  #   #
  #   #   #   #   #   #  #   #  #   #
  #    ####   #   #   #   ####  #### 
  #       #   #   #   #      #  #   #
  #   #   #   #   #   #  #   #  #   #
 ###   ###   ###   ###    ###   #### 
*/

#include "main.h"
#include "lemlib/api.hpp"

#define SCORE_PORT 10
#define INTAKE_PORT 6
#define CONTAINER_PORT 9
#define RIGHT1 18
#define RIGHT2 19
#define RIGHT3 20
#define LEFT1 13
#define LEFT2 12
#define LEFT3 11
#define IMU_PORT 14
#define H_ENCODER_PORT -16 
#define V_ENCODER_PORT -17 

#define TRACK_WIDTH 10.5
#define HORIZONTAL_DRIFT 4
#define H_TRACKING_OFFSET -3
#define V_TRACKING_OFFSET 0


const int DRIVE_RPM = 450;

pros::Motor score(SCORE_PORT);
pros::Motor intake(INTAKE_PORT);
pros::Motor container(CONTAINER_PORT);
pros::Motor right1(RIGHT1);
pros::Motor right2(RIGHT2);
pros::Motor right3(RIGHT3);
pros::Motor left1(LEFT1);
pros::Motor left2(LEFT2);
pros::Motor left3(LEFT3);

pros::adi::DigitalOut arms ('G');
pros::adi::DigitalOut matchloader ('H');

pros::MotorGroup left_mg({-LEFT1, -LEFT2, -LEFT3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({RIGHT1, RIGHT2, RIGHT3});

// imu
pros::Imu imu(IMU_PORT);
pros::Rotation h_encoder(H_ENCODER_PORT);
pros::Rotation v_encoder(V_ENCODER_PORT);
lemlib::TrackingWheel h_tracking_wheel(&h_encoder, lemlib::Omniwheel::NEW_2, H_TRACKING_OFFSET);
lemlib::TrackingWheel v_tracking_wheel(&v_encoder, lemlib::Omniwheel::NEW_2, V_TRACKING_OFFSET);

pros::Controller controller (pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              TRACK_WIDTH, 
                              lemlib::Omniwheel::NEW_275,
                              DRIVE_RPM, // drivetrain rpm is 450 (600 RPM Motor x 36T gear / 48T wheel)
                              HORIZONTAL_DRIFT
);

// odometry settings
lemlib::OdomSensors sensors(&v_tracking_wheel,
                            nullptr,
                            &h_tracking_wheel,
                            nullptr,
                            &imu 
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(1.5, // proportional gain (kP)
                                              0.001, // integral gain (kI)
                                              25, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              127 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.3, // proportional gain (kP)
                                              0.005, // integral gain (kI)
                                              2.75, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              127 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.04 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.09 // expo curve gain
);


lemlib::Chassis chassis(drivetrain, 
                        lateral_controller, 
                        angular_controller, 
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

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
    // chassis.moveToPoint(0,15,2000, {.minSpeed = 100});
    chassis.setPose(0,0,0);
    intake.move_velocity(200);
    chassis.moveToPoint(0, 12, 1000);
    chassis.turnToHeading(10, 1000);
    chassis.moveToPoint(-15,50, 2000, {.minSpeed = 100});
    pros::delay(500);
    intake.move_velocity(0);
    chassis.turnToHeading(100, 1000);
    chassis.moveToPoint(-48, 20, 2000, {.minSpeed = 100});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-48, 50, 2000, {.forwards = false, .minSpeed = 100});
    intake.move_velocity(200);
    pros::delay(200);
    intake.move_velocity(-200);
    pros::delay(200);
    intake.move_velocity(200);
    pros::delay(200);
    intake.move_velocity(-200);
    pros::delay(200);
    intake.move_velocity(200);
    pros::delay(2500);
    intake.move_velocity(0);

    // chassis.moveToPoint(0,12,2000);
    // chassis.moveToPoint(-12,12,2000);
    // chassis.turnToHeading(0,1000);

    // matchloader.set_value(true);
    // intake.move_velocity(200);

    // chassis.moveToPoint(-12,-12,4000);
    
    // chassis.moveToPoint(-12,24,4000);
    // arms.set_value(false);
    // intake.move_velocity(0);

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
