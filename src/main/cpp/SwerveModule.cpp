#include "SwerveModule.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int turnID, int encoderID, std::string pos, double offset, double kP, double kI, double kD)
    : m_rotate{turnID, "drivebase"}, m_speed{driveID, "drivebase"}, m_encoder{encoderID, "drivebase"}, m_wheelName{pos}, m_offset{offset}, m_pid{kP, kI, kD}
{
    m_encoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    // m_rotate.SetNeutralMode(NeutralMode::Brake);
};

/**
 * Robot actually go brr 🚗
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
    double error = fmod(angle - m_currentAngle, 360);
    // m_shuffError->SetDouble(error);
    frc::SmartDashboard::PutNumber(m_wheelName + " Error", error);
    if (error > 180)
    {
        error = -(360 - error);
    }
    else if (error < -180)
    {
        error = 360 + error;
    }
    // if the target angle is the opposite of the current angle (m_currentAngle)
    if (abs(m_currentAngle - m_prevAngle) == 180)
    {
        voltage *= -1;
    }
    else
    {
        double turn = m_pid.calculate(error);
        m_rotate.SetVoltage(units::volt_t{turn});
    }
    m_speed.SetVoltage(units::volt_t{voltage});
}

void SwerveModule::periodic()
{
    m_currentAngle = m_encoder.GetAbsolutePosition();
    m_currentAngle -= m_offset;
    // m_shuffEncoderAngle->SetDouble(m_currentAngle);
    frc::SmartDashboard::PutNumber(m_wheelName + " Angle", m_currentAngle);
    // if (m_wheelName == "FR")
    // {
    //     std::cout << "pid: " << m_kP->GetDouble(0) << " " << m_kI->GetDouble(0) << " " << m_kP->GetDouble(0);
    //     m_pid.setValues(m_kP->GetDouble(0), m_kI->GetDouble(0), m_kP->GetDouble(0));
    // }
}
