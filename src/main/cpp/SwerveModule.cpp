#include "SwerveModule.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int turnID, int encoderID, std::string pos, double offset, double kP, double kI, double kD)
    : m_rotate{turnID, "drivebase"}, m_speed{driveID, "drivebase"}, m_encoder{encoderID, "drivebase"}, m_wheelName{pos}, m_offset{offset}, m_pid{kP, kI, kD}
{
    m_encoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    // m_rotate.SetNeutralMode(NeutralMode::Brake);
};

double SwerveModule::optimizeRotation(double angle)
{
    if (angle > 180)
    {
        angle = -(360 - angle);
    }
    else if (angle < -180)
    {
        angle = 360 + angle;
    }
    return angle;
}

/**
 * Returns the optimal degres required to get to a target
 */
double SwerveModule::angleToTarget(double targetAngle)
{
    return optimizeRotation(targetAngle - m_currentAngle);
}

/**
 * Drive a swerve module
 * @param angle angle to turn to
 * @param voltage voltage to drive at
 */
void SwerveModule::driveAt(double angle, double voltage)
{
    frc::SmartDashboard::PutNumber(m_wheelName + " Target Angle", angle);
    if (angle == 0)
    {
        angle = m_prevAngle;
    }
    else
    {
        m_prevAngle = angle;
    }
    if (abs(voltage) > 5)
    {
        voltage = 5;
    }
    double angle_by_rotation = angleToTarget(angle);
    frc::SmartDashboard::PutNumber(m_wheelName + " Error", angle_by_rotation);
    double angle_to_parallel = angleToTarget(angle - 180);
    // if it takes less rotation to get parallel to the target angle than rotating fully
    // flip the drive voltage to go in the opposite way
    if (abs(angle_to_parallel) < abs(angle_by_rotation))
    {
        double turn = m_pid.calculate(angle_to_parallel);
        m_rotate.SetVoltage(units::volt_t{turn});
        m_speed.SetVoltage(units::volt_t{-voltage});
    }
    else
    {
        double turn = m_pid.calculate(angle_by_rotation);
        m_rotate.SetVoltage(units::volt_t{turn});
        m_speed.SetVoltage(units::volt_t{voltage});
    }
}

void SwerveModule::periodic()
{
    m_currentAngle = m_encoder.GetAbsolutePosition();
    m_currentAngle -= m_offset;
    frc::SmartDashboard::PutNumber(m_wheelName + " Angle", m_currentAngle);
}
