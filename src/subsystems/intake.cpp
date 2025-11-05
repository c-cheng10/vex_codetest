#include "robot/intake.h"

#include "globals.h"
#include "lemlib/timer.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

#define INTAKE_SPEED 200
#define SCORE_SPEED 200

using namespace Robot;
using namespace Robot::Globals;

void Intake::run() {
    int intake_state = 0;
    int score_state = 0;

    // --- Intake control ---
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        intake_state = 1;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake_state = -1;
    } else {
        intake_state = 0;
    }

    // --- Scoring control ---
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        score_state = 1;
        intake_state = 1;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        score_state = -1;
        intake_state = 1;
    } else {
        score_state = 0;
    }

    // --- Intake motor control ---
    if (intake_state == 1) {
        intake.move_velocity(INTAKE_SPEED);
        container.move_velocity(-INTAKE_SPEED);
    } else if (intake_state == -1) {
        intake.move_velocity(-INTAKE_SPEED);
        container.move_velocity(INTAKE_SPEED);
    } else {
        intake.move_velocity(0);
        container.move_velocity(0);
    }

    // --- Scoring motor control ---
    if (score_state == 1) {
        score.move_velocity(SCORE_SPEED);
    } else if (score_state == -1) {
        score.move_velocity(-SCORE_SPEED);
    } else {
        score.move_velocity(0);
    }
}
