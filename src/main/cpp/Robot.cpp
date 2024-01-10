// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit()
{
    try
    {
        m_navx = new AHRS(frc::SerialPort::Port::kMXP);
    }
    catch (const std::exception &e)
    {
    }
}
void Robot::RobotPeriodic()
{
    // log encoders
    m_swerve.periodic();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
    m_swerve.driveRobot(m_joystickSpeed.GetX(), -m_joystickSpeed.GetY(), m_joystickSpin.GetX());
    m_swerve.setrot(m_navx->GetYaw());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
