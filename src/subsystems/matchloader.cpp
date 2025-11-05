#include "robot/matchloader.h"

#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

void Matchloader::run() {
   if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      Matchloader::toggle();
   }
}

Matchloader::Matchloader() { }

void Matchloader::toggle() { matchloader.toggle(); }