#include "SwerveModule.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int turnID, int encoderID, std::string pos, double offset, double kP, double kI, double kD)
    : m_rotate{turnID, "drivebase"}, m_speed{driveID, "drivebase"}, m_encoder{encoderID, "drivebase"}, m_wheelName{pos}, m_offset{offset}, m_pid{PID{kP, kI, kD}}
{
    m_encoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    if (pos == "FR")
    {
        frc::ShuffleboardTab *m_tab = &frc::Shuffleboard::GetTab("Drivebase");
        m_kP = m_tab->Add("kP", 0).GetEntry();
        m_kI = m_tab->Add("kI", 0).GetEntry();
        m_kD = m_tab->Add("kD", 0).GetEntry();
    }
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
    m_currentAngle -= m_offset;
    // m_shuffEncoderAngle->SetDouble(m_currentAngle);
    frc::SmartDashboard::PutNumber(m_wheelName + " Angle", m_currentAngle);
    // if (m_wheelName == "FR")
    // {
    //     std::cout << "pid: " << m_kP->GetDouble(0) << " " << m_kI->GetDouble(0) << " " << m_kP->GetDouble(0);
    //     m_pid.setValues(m_kP->GetDouble(0), m_kI->GetDouble(0), m_kP->GetDouble(0));
    // }
}
