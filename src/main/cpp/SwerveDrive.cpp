#include "SwerveDrive.h"
#include <cmath>

const int DEADBAND = 0.1;

/**
 * 🚗 robot go brr but like it controls all the modules
 * @param x x input from the joystick
 * @param y y input from the joystick
 * @param rot rotation speed from the joystick (x)
 */
void SwerveDrive::driveRobot(double x, double y, double rot)
{
    frc::SmartDashboard::PutNumber("Joy1 X", x);
    frc::SmartDashboard::PutNumber("Joy1 Y", y);
    frc::SmartDashboard::PutNumber("Joy2 Rot", rot);
    if (std::abs(x) < DEADBAND)
    {
        x = 0;
    }
    if (std::abs(y) < DEADBAND)
    {
        y = 0;
    }
    if (std::abs(rot) < DEADBAND)
    {
        rot = 0;
    }
    vector fr = calculate(x, y, rot, -1, 1);
    fr.log("FR");
    m_moduleFR.driveAt(fr.angle, fr.magnitude);

    vector br = calculate(x, y, rot, 1, 1);
    br.log("BR");
    m_moduleBR.driveAt(br.angle, br.magnitude);

    vector fl = calculate(x, y, rot, -1, -1);
    fl.log("FL");
    m_moduleFL.driveAt(fl.angle, fl.magnitude);

    vector bl = calculate(x, y, rot, 1, -1);
    bl.log("BL");
    m_moduleBL.driveAt(bl.angle, bl.magnitude);
}

/**
 * Calculate vector
 * @param x x input from the joystick
 * @param y y input from the joystick
 * @param rot rotation speed from the joystick (x) -- positive rot is spin robot counterclockwise, negative rot is spin robot clockwise
 * @param a drive module x dir
 * @param b drive module y dir
 */
SwerveDrive::vector SwerveDrive::calculate(double x, double y, double rot, int a, int b)
{
    // field orient
    // x = x * cos(m_currentangle) - y * sin(m_currentangle);
    // y = x * sin(m_currentangle) + y * cos(m_currentangle);

    double x_mag = x + rot * a;
    double y_mag = y + rot * b;
    double mag = hypot(x_mag, y_mag);
    double angle = atan2(y_mag, x_mag);
    angle = angle * 180 / M_PI;
    return vector{angle, mag};
}

void SwerveDrive::periodic()
{
    m_moduleBL.periodic();
    m_moduleBR.periodic();
    m_moduleFL.periodic();
    m_moduleFR.periodic();
}

void SwerveDrive::setrot(double angle)
{
    m_currentangle = angle;
    frc::SmartDashboard::PutNumber("Robot Angle", m_currentangle);
}

void SwerveDrive::reset()
{
    m_currentangle = 0;
    frc::SmartDashboard::PutNumber("Robot Angle", m_currentangle);
}
