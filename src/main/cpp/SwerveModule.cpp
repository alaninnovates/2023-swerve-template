#include "SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int turnID, int encoderID, std::string pos, double offset)
    : m_rotate{turnID, "drivebase"}, m_speed{driveID, "drivebase"}, m_encoder{encoderID, "drivebase"}, m_wheelName{pos}, m_offset{offset}
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
    if (abs(voltage) > 5)
    {
        voltage = 5;
    }
    m_speed.SetVoltage(units::volt_t{voltage});
    double error = angle - m_currentAngle;
    frc::SmartDashboard::PutNumber(m_wheelName + " error", error);
    if (error > 180)
    {
        error = -(error - 180);
    }
    if (error < -180)
    {
        error = -(error + 180);
    }
    double kP = 0.01;
    double turn = kP * error;
    m_rotate.SetVoltage(units::volt_t{turn});
}

void SwerveModule::periodic()
{
    m_currentAngle = m_encoder.GetAbsolutePosition();
    m_currentAngle += m_offset;
    frc::SmartDashboard::PutNumber(m_wheelName + " encoder angle", m_currentAngle);
}
