#include "PID.h"

PID::PID(double kP, double kI, double kD)
    : m_kP{kP}, m_kI{kI}, m_kD{kD}, m_error{0}, m_integral{0}, m_derivative{0}, m_previous_error{0} {};

double PID::calculate(double error)
{
    m_error = error;
    // 20ms periodic loop
    m_integral += m_error * 0.02;
    m_derivative = (m_error - m_previous_error) / 0.02;
    m_previous_error = m_error;
    double output = 0;
    output += m_kP * m_error;
    output += m_kI * m_integral;
    output += m_kD * m_derivative;
    return output;
}

void PID::reset()
{
    m_error = 0;
    m_integral = 0;
    m_derivative = 0;
    m_previous_error = 0;
}
