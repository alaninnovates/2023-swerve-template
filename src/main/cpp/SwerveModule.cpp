#include "SwerveModule.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int turnID, int encoderID, std::string pos, double offset, double kP, double kI, double kD)
    : m_rotate{turnID, "drivebase"}, m_speed{driveID, "drivebase"}, m_encoder{encoderID, "drivebase"}, m_wheelName{pos}, m_offset{offset}, m_pid{PID{kP, kI, kD}}
{
    m_encoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
};

/**
 * Robot actually go brr 🚗
 * @param angle angle to turn to
 * @param voltage voltage to drive at
 */
void SwerveModule::driveAt(double angle, double voltage)
{
    m_shuffVectorAngle->SetDouble(angle);
    m_shuffVectorMagnitude->SetDouble(voltage);
    if (m_shuffOverrideEnabled->GetBoolean(false))
    {
        angle = m_shuffDebugAngle->GetDouble(0);
        voltage = m_shuffDebugMagnitude->GetDouble(0);
    }
    if (abs(voltage) > 5)
    {
        voltage = 5;
    }
    m_speed.SetVoltage(units::volt_t{voltage});
    double error = fmod(angle - m_currentAngle, 360);
    m_shuffError->SetDouble(error);
    frc::SmartDashboard::PutNumber(m_wheelName + " Error", error);
    if (error > 180)
    {
        error = -(error - 180);
    }
    if (error < -180)
    {
        error = -(error + 180);
    }
    double turn = m_pid.calculate(error);
    m_rotate.SetVoltage(units::volt_t{turn});
}

void SwerveModule::periodic()
{
    m_currentAngle = m_encoder.GetAbsolutePosition();
    m_currentAngle += m_offset;
    // m_shuffEncoderAngle->SetDouble(m_currentAngle);
    frc::SmartDashboard::PutNumber(m_wheelName + " Angle", m_currentAngle);
}
