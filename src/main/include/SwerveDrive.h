#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "SwerveModule.h"
#include "Constants.h"
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class SwerveDrive
{
public:
    void driveRobot(double xval, double yvel, double rotspeed);
    void reset();
    void setrot(double angle);
    void periodic();

private:
    struct vector
    {
        double angle;
        double magnitude;
        void log(std::string name)
        {
            frc::SmartDashboard::PutNumber(name + " Angle", angle);
            frc::SmartDashboard::PutNumber(name + " Magnitude", magnitude);
        }
    };
    vector calculate(double xvel, double yvel, double rotspeed, int a, int b);
    double m_currentangle;
    SwerveModule m_moduleFR{frDM, frTM, frENC, "FR"};
    SwerveModule m_moduleBR{brDM, brTM, brENC, "BR"};
    SwerveModule m_moduleFL{flDM, flTM, flENC, "FL"};
    SwerveModule m_moduleBL{blDM, blTM, blENC, "BL"};
};