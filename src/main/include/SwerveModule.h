#pragma once
#include <cmath>
#include <ctre/Phoenix.h>

class SwerveModule
{
public:
    SwerveModule(int driveID, int turnID, int encoderID, std::string pos);
    void driveAt(double angle, double voltage);
    void periodic();

private:
    WPI_TalonFX m_rotate;
    WPI_TalonFX m_speed;
    WPI_CANCoder m_encoder;
    double m_currentangle;
    std::string pos;
};