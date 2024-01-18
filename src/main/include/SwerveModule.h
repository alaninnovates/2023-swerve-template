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
    double optimizeRotation(double angle);
    double angleToTarget(double targetAngle);
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
};