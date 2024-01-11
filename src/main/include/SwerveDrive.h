#include <frc/shuffleboard/Shuffleboard.h>
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
    };
    vector calculate(double xvel, double yvel, double rotspeed, int a, int b);
    double m_currentAngle;
    double m_targetAngle;
    SwerveModule m_moduleFR{frDM, frTM, frENC, "FR", frOffset};
    SwerveModule m_moduleBR{brDM, brTM, brENC, "BR", brOffset};
    SwerveModule m_moduleFL{flDM, flTM, flENC, "FL", flOffset};
    SwerveModule m_moduleBL{blDM, blTM, blENC, "BL", blOffset};
    frc::ShuffleboardTab *tab = &frc::Shuffleboard::GetTab("Drivebase");
    frc::ShuffleboardLayout *m_shuffJoystick = &tab->GetLayout("Joystick", frc::BuiltInLayouts::kList);
    nt::GenericEntry *m_shuffJoyX = m_shuffJoystick->Add("Joy1 X", 0).GetEntry();
    nt::GenericEntry *m_shuffJoyY = m_shuffJoystick->Add("Joy1 Y", 0).GetEntry();
    nt::GenericEntry *m_shuffJoyRot = m_shuffJoystick->Add("Joy2 Rot", 0).GetEntry();
    nt::GenericEntry *m_shuffRobotAngle = tab->Add("Robot Angle", 0).GetEntry();
};