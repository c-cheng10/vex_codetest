#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot/intake.h"
#include <iostream>
#include <string>

using namespace Robot;
using namespace Robot::Globals;

class auton{
    public:
        static void leftAWP(){
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0,20,4000);
            chassis.turnToHeading(-90, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});

            // face towards left

            // forward
            // left 90
            // forward
            // get 3 alliance blocks
            // back towards goal and score 3 - 4
            // turn around and get 3 more
            // score 1 - 3 in the upper middle goal
            // intake more blocks
            // score in lower middle goal
        }

        static void rightAWP(){
            // face towards right
            // forward
            // right 90
            // forward
            // get 3 alliance blocks
            // back towards goal and score 3 - 4
            
        }

        static void leftNormal(){
            // left normal autonomous code
        }

        static void rightNormal(){
            // right normal autonomous code
        }
};