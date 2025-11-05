#include "robot/descore.h"

#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

void Descore::run() {
   if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      Descore::toggle();
   }
}

Descore::Descore() { }

void Descore::toggle() { descore.toggle(); }