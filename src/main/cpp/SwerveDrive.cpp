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
    frc::ShuffleboardLayout &layout = m_tab.GetLayout("Joystick", frc::BuiltInLayouts::kList);
    layout.Add("Joy1 X", x);
    layout.Add("Joy1 Y", y);
    layout.Add("Joy2 Rot", rot);
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
    vector fr = calculate(x, y, rot, -1, 1, "FR");
    fr.log("FR");
    fr.override_from_shuffleboard("FR");
    m_moduleFR.driveAt(fr.angle, fr.magnitude);

    vector fl = calculate(x, y, rot, -1, -1, "FL");
    fl.log("FL");
    fl.override_from_shuffleboard("FL");
    m_moduleFL.driveAt(fl.angle, fl.magnitude);

    vector br = calculate(x, y, rot, 1, 1, "BR");
    br.log("BR");
    br.override_from_shuffleboard("BR");
    m_moduleBR.driveAt(br.angle, br.magnitude);

    vector bl = calculate(x, y, rot, 1, -1, "BL");
    bl.log("BL");
    bl.override_from_shuffleboard("BL");
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
SwerveDrive::vector SwerveDrive::calculate(double x, double y, double rot, int a, int b, std::string name)
{
    // todo:field orient
    double x_mag = 6 * (x + rot * a);
    double y_mag = 6 * (y + rot * b);

    frc::ShuffleboardTab &m_tab = frc::Shuffleboard::GetTab("Drivebase");
    frc::ShuffleboardLayout &layout = m_tab.GetLayout(name + " Module", frc::BuiltInLayouts::kList);
    layout.Add("x_mag", x_mag);
    layout.Add("y_mag", y_mag);

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
    m_tab.Add("Robot Angle", m_currentAngle);
}

void SwerveDrive::reset()
{
    m_currentAngle = 0;
    m_tab.Add("Robot Angle", m_currentAngle);
}
