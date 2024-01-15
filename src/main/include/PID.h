class PID
{
public:
    PID(double kP, double kI, double kD);
    double calculate(double error);
    void reset();

private:
    double m_kP;
    double m_kI;
    double m_kD;
    double m_error;
    double m_integral;
    double m_derivative;
    double m_previous_error;
};