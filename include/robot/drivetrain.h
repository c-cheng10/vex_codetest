#pragma once
#include <string>

namespace Robot {

class Drivetrain {
public:
    Drivetrain();

    enum DRIVE_MODE { ARCADE_DRIVE = 0 };

    static DRIVE_MODE driveMode;

    void run();

    static std::string toggleDrive();
    
    static int thrustHandler(int thrust);

    static bool isReversed;

private:
    void ArcadeDrive();
};

}
