#include "robot/drivetrain.h"
#include "globals.h"
#include <cmath>
#define DEFAULT_DELAY_LENGTH 20

using namespace Robot;
using namespace Robot::Globals;

Drivetrain::DRIVE_MODE Drivetrain::driveMode = ARCADE_DRIVE;

bool Drivetrain::isReversed = false;

Drivetrain::Drivetrain() {}


void Drivetrain::ArcadeDrive() {
   int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
   int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
   if (throttle <= 1) {
      turn = std::round(arcade_turn_curve.curve(turn));
   }

   chassis.arcade(thrustHandler(throttle), thrustHandler(turn), false, 0.55);

   pros::delay(DEFAULT_DELAY_LENGTH);
}


void Drivetrain::run() {
   switch (Drivetrain::driveMode) {
   case ARCADE_DRIVE:
      Drivetrain::ArcadeDrive();
      break;
   }
}


int Drivetrain::thrustHandler(int thrust) {
   if (Drivetrain::isReversed) {
      return thrust * -1;
   }
   return thrust;
}
