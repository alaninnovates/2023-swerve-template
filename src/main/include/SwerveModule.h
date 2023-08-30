#pragma once
#include <ctre/Phoenix.h>
#include <cmath>

class SwerveModule{
  public:
    SwerveModule(int driveID, int turnID, int encoderID);
    void driveGO(double angle, double voltage);
    void periodic();
  private:
    WPI_TalonFX m_rotate;
    WPI_TalonFX m_speed;
    WPI_CANCoder m_encoder;
    double currentangle;
};