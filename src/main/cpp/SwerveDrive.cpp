#include <SwerveDrive.h>
#include <cmath>

void SwerveDrive::driveRobot(double xvel, double yvel, double rotspeed){
    //TODO: Call calculate function based on rotation and vector and pass to each swerve module
    return;
}

SwerveDrive::vector SwerveDrive::calculate(double xvel, double yvel, double rotspeed, int a, int b){
    //TODO Calclulate vector using math
    vector vector1;
    // vector1 is the result vector
    return vector1;
}

void SwerveDrive::setrot(double angle){
    currentangle = angle;
}

void SwerveDrive::reset(){
    //TODO set everything to 0
    return;
}