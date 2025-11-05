#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"
#include "lemlib/api.hpp"

namespace Robot {
    namespace Globals {

        extern pros::Controller controller;

        extern pros::Motor right1;
        extern pros::Motor right2;
        extern pros::Motor right3;
        extern pros::Motor left1;
        extern pros::Motor left2;
        extern pros::Motor left3;

        extern pros::Motor score;
        extern pros::Motor intake;
        extern pros::Motor container;

        extern pros::adi::Pneumatics descore;
        extern pros::adi::Pneumatics matchloader;

        extern pros::Imu imu;
        extern pros::Rotation h_encoder;
        extern pros::Rotation v_encoder;
        extern lemlib::TrackingWheel h_tracking_wheel;
        extern lemlib::TrackingWheel v_tracking_wheel;

        extern pros::MotorGroup left_mg;
        extern pros::MotorGroup right_mg;

        extern lemlib::Drivetrain drivetrain;
        extern lemlib::OdomSensors sensors;

        extern lemlib::ControllerSettings lateral_controller;
        extern lemlib::ControllerSettings angular_controller;

        extern lemlib::ExpoDriveCurve throttle_curve;
        extern lemlib::ExpoDriveCurve steer_curve;

        extern lemlib::Chassis chassis;

    }
}
