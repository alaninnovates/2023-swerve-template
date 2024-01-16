#include "SwerveDrive.h"
#include <cmath>

const double DEADBAND = 0.1;

/**
 * 🚗 robot go brr but like it controls all the modules
 * @param x x input from the joystick
 * @param y y input from the joystick
 * @param rot rotation speed from the joystick (x)
 */
void SwerveDrive::driveRobot(double x, double y, double rot)
{
    m_shuffJoyX->SetDouble(x);
    m_shuffJoyY->SetDouble(y);
    m_shuffJoyRot->SetDouble(rot);
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
    vector fr = calculate(x, y, rot, -1, -1);
    m_moduleFR.driveAt(fr.angle, fr.magnitude);

    vector fl = calculate(x, y, rot, -1, 1);
    m_moduleFL.driveAt(fl.angle, fl.magnitude);

    vector br = calculate(x, y, rot, 1, -1);
    m_moduleBR.driveAt(br.angle, br.magnitude);

    vector bl = calculate(x, y, rot, 1, 1);
    m_moduleBL.driveAt(bl.angle, bl.magnitude);

    m_targetAngle = atan2(y, x) * 180 / M_PI;
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
    // todo:field orient
    double x_mag = 6 * (x + rot * a);
    double y_mag = 6 * (y + rot * b);

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
    // drift correction
    // if (m_currentAngle != m_targetAngle)
    // {
    //     double diff = m_targetAngle - m_currentAngle;
    //     driveRobot(0, 0, diff * 0.01);
    // }
}

void SwerveDrive::setrot(double angle)
{
    m_currentAngle = angle;
    m_shuffRobotAngle->SetDouble(m_currentAngle);
}

void SwerveDrive::reset()
{
    m_currentAngle = 0;
    m_shuffRobotAngle->SetDouble(m_currentAngle);
}
