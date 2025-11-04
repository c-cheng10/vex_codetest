#include "main.h"
#include "lemlib/api.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"

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

        pros::Imu imu(IMU_PORT);
        pros::Rotation h_encoder(H_ENCODER_PORT);
        pros::Rotation v_encoder(V_ENCODER_PORT);
        lemlib::TrackingWheel h_tracking_wheel(&h_encoder, lemlib::Omniwheel::NEW_2, -3);
        lemlib::TrackingWheel v_tracking_wheel(&v_encoder, lemlib::Omniwheel::NEW_2, 0);

        pros::MotorGroup left_mg({left1.get_port(), left2.get_port(), left3.get_port()});
        pros::MotorGroup right_mg({RIGHT1, RIGHT2, RIGHT3});

    }
}