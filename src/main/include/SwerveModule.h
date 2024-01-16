#pragma once
#include <cmath>
#include <ctre/Phoenix.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include "PID.h"

class SwerveModule
{
public:
    SwerveModule(int driveID, int turnID, int encoderID, std::string pos, double offset, double kP, double kI, double kD);
    void driveAt(double angle, double voltage);
    void periodic();

private:
    WPI_TalonFX m_rotate;
    WPI_TalonFX m_speed;
    WPI_CANCoder m_encoder;
    double m_currentAngle;
    double m_prevAngle;
    double m_offset;
    std::string m_wheelName;
    PID m_pid;
    frc::ShuffleboardLayout *layout = &frc::Shuffleboard::GetTab("Drivebase")
                                           .GetLayout(m_wheelName + " Module", frc::BuiltInLayouts::kList)
                                           .WithSize(2, 5);
    nt::GenericEntry *m_shuffEncoderAngle = layout->Add("Encoder Angle", 0).WithPosition(5, 0).GetEntry();
    nt::GenericEntry *m_shuffError = layout->Add("Error", 0).WithPosition(4, 0).GetEntry();
    nt::GenericEntry *m_shuffVectorAngle = layout->Add("Vector Angle", 0).WithPosition(3, 0).GetEntry();
    nt::GenericEntry *m_shuffVectorMagnitude = layout->Add("Vector Magnitude", 0).WithPosition(4, 0).GetEntry();
    nt::GenericEntry *m_shuffDebugAngle = layout->Add("[DBG] Angle", 1)
                                              .WithWidget(frc::BuiltInWidgets::kDial)
                                              .WithProperties({
                                                  {"min", nt::Value::MakeDouble(0)},
                                                  {"max", nt::Value::MakeDouble(360)},
                                              })
                                              .WithPosition(1, 0)
                                              .GetEntry();
    nt::GenericEntry *m_shuffDebugMagnitude = layout->Add("[DBG] Magnitude", 1)
                                                  .WithWidget(frc::BuiltInWidgets::kDial)
                                                  .WithProperties({
                                                      {"min", nt::Value::MakeDouble(0)},
                                                      {"max", nt::Value::MakeDouble(1)},
                                                  })
                                                  .WithPosition(2, 0)
                                                  .GetEntry();
    nt::GenericEntry *m_shuffOverrideEnabled = layout->Add("[DBG] Override Enabled", false).WithWidget(frc::BuiltInWidgets::kToggleButton).WithPosition(0, 0).GetEntry();
    nt::GenericEntry *m_kP, *m_kI, *m_kD;
};