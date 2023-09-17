#include "SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int turnID, int encoderID, std::string pos)
    : m_rotate{turnID, "drivebase"}, m_speed{driveID, "drivebase"}, m_encoder{encoderID, "drivebase"}, pos{pos}
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
    double error = angle - m_currentangle;
    frc::SmartDashboard::PutNumber(pos + " error", error);
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
    m_currentangle = m_encoder.GetAbsolutePosition();
    frc::SmartDashboard::PutNumber(pos + " encoder angle", m_currentangle);
}
