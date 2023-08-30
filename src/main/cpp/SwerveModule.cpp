#include <SwerveModule.h>

SwerveModule::SwerveModule(int driveID, int turnID, int encoderID): m_rotate{turnID, "Drivebase"}, m_speed{driveID, "Drivebase"}, m_encoder{encoderID, "Drive"} {};

void driveGO(double angle, double voltage){
    //tell turnmotor to turn to an angle and then tell drivemotor to move
}

void periodic(){
    //set the current angle based off of encoder
}