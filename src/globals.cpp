#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"

namespace Robot {

    namespace Globals {

        pros::Controller controller (pros::E_CONTROLLER_MASTER);

        pros::Motor right1(18, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        pros::Motor right2(19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        pros::Motor right3(20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        pros::Motor left1(-13, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        pros::Motor left2(-12, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        pros::Motor left3(-11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

        pros::Motor score(10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        pros::Motor intake(6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        pros::Motor container(9, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

        pros::adi::Pneumatics descore('G', false);
        pros::adi::Pneumatics matchloader('H', false);

        pros::Imu imu(14);
        pros::Rotation h_encoder(-16);
        pros::Rotation v_encoder(-17);
        lemlib::TrackingWheel h_tracking_wheel(&h_encoder, lemlib::Omniwheel::NEW_2, -3);
        lemlib::TrackingWheel v_tracking_wheel(&v_encoder, lemlib::Omniwheel::NEW_2, 0);

        pros::MotorGroup left_mg({left1.get_port(), left2.get_port(), left3.get_port()});
        pros::MotorGroup right_mg({right1.get_port(), right2.get_port(), right3.get_port()});

        lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              10.5, //track width
                              lemlib::Omniwheel::NEW_275,
                              450, // drivetrain rpm is 450 (600 RPM Motor x 36T gear / 48T wheel)
                              2 //horizontal drift
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

        lemlib::ExpoDriveCurve arcade_turn_curve(1,   // joystick deadband out of 127
                                   8,  // minimum output where drivetrain will move out of 127
                                   1.016 // expo curve gain
);

        lemlib::Chassis chassis(drivetrain, 
                                lateral_controller, 
                                angular_controller, 
                                sensors,
                                &throttle_curve, 
                                &steer_curve
        );

    }
}