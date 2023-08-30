#include <ctre/Phoenix.h>
#include "SwerveModule.h"

class SwerveDrive {
  public:
    void driveRobot(double xval, double yvel, double rotspeed);
    void reset();
    void setrot(double angle);
  private:
    struct vector{
        double angle;
        double magnitude;
    };
    vector calculate(double xvel, double yvel, double rotspeed, int a, int b);
    double currentangle;
};