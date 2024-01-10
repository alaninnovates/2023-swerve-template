#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/Phoenix.h>
#include "SwerveModule.h"
#include "Constants.h"
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class SwerveDrive
{
public:
    void driveRobot(double xval, double yvel, double rotspeed);
    void reset();
    void setrot(double angle);
    void periodic();

private:
    struct vector
    {
        double angle;
        double magnitude;
        void log(std::string name)
        {
            frc::ShuffleboardTab &m_tab = frc::Shuffleboard::GetTab("Drivebase");
            frc::ShuffleboardLayout &layout = m_tab.GetLayout(name + " Module", frc::BuiltInLayouts::kList);
            layout.Add("Angle", angle);
            layout.Add("Magnitude", magnitude);
        }
        void override_from_shuffleboard(std::string name)
        {
            frc::ShuffleboardTab &m_tab = frc::Shuffleboard::GetTab("Drivebase");
            frc::ShuffleboardLayout &layout = m_tab.GetLayout(name + " Module", frc::BuiltInLayouts::kList);
            bool override_enabled = layout.Add("Override Enabled", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();
            if (!override_enabled)
            {
                return;
            }
            angle = layout.Add("[DBG] Angle", 1)
                        .WithWidget(frc::BuiltInWidgets::kDial)
                        .WithProperties({
                            {"min", nt::Value::MakeDouble(0)},
                            {"max", nt::Value::MakeDouble(360)},
                        })
                        .GetEntry()
                        ->GetDouble(0.0);
            magnitude = layout.Add("[DBG] Magnitude", 1)
                            .WithWidget(frc::BuiltInWidgets::kDial)
                            .WithProperties({
                                {"min", nt::Value::MakeDouble(0)},
                                {"max", nt::Value::MakeDouble(1)},
                            })
                            .GetEntry()
                            ->GetDouble(0.0);
        }
    };
    vector calculate(double xvel, double yvel, double rotspeed, int a, int b, std::string name);
    double m_currentAngle;
    double m_targetAngle;
    SwerveModule m_moduleFR{frDM, frTM, frENC, "FR", frOffset};
    SwerveModule m_moduleBR{brDM, brTM, brENC, "BR", brOffset};
    SwerveModule m_moduleFL{flDM, flTM, flENC, "FL", flOffset};
    SwerveModule m_moduleBL{blDM, blTM, blENC, "BL", blOffset};
    frc::ShuffleboardTab &m_tab = frc::Shuffleboard::GetTab("Drivebase");
};