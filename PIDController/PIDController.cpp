/*
Create basic PID Controller class. 

Author: Felix Li 
Last Updated: 9/15/2018
*/

/**
    Main function to return end PID term. 
    @param: measurement. Latest measurement. 
    @return: PID term.
*/

/////////////////////////////
//  Header Files
/////////////////////////////
#include "PIDController.h" 
#include "HardwareSerial.h" //for debugging

/**
    Core function of PID controller. Based on new measuremnet, calculates new PID term.

    @param measurement. Latest measurement.
    @param dt. Delta time since last iteration.
    @return. Returns PID term.
*/
float PIDController::update(float measurement, float dt) {

    //set up for p, d, and i respectively 
    float error = _ref - measurement;
    float deltaError = (error - _prevError)/dt;
    _totalError += (error)*dt;

    //make sure accumulated error does not go out of bounds
    if (_totalError > _constraint) _totalError = _constraint; 
    if (_totalError < -_constraint) _totalError = -_constraint;

    float pid = _kp * error + _kd * deltaError + _ki*_totalError;
    if (pid > 0) pid += _bias;
    if (pid < 0) pid -= _bias; 

    #ifdef DEBUG_PID
    Serial.println("PID Controller:");
    Serial.print("\t"); Serial.print("New - "); Serial.println(measurement); 
    Serial.print("\t"); Serial.print("Error - "); Serial.println(error); 
    Serial.print("\t"); Serial.print("Delta - "); Serial.println(deltaError);
    Serial.print("\t"); Serial.print("Total - "); Serial.println(_totalError);
    Serial.print("\t"); Serial.print("Output - "); Serial.println(pid);
    #endif

    //update terms for next iteration
    _prevError = error;

    //quick enhancement. If error really close to 0. reset integral term. (otherwise accumulates forever) 
    // if (error < 1.5f and error > -1.5f) { //no need to import abs
    //     _totalError = 0;
    // }

    return pid; 

}

/**
    Print out internal values. 
*/
void PIDController::printMe()
{
    Serial.println("PID Controller Values:");
    Serial.print("\t"); Serial.print("kp - "); Serial.println(_kp); 
    Serial.print("\t"); Serial.print("ki - "); Serial.println(_ki); 
    Serial.print("\t"); Serial.print("kd - "); Serial.println(_kd);
    Serial.print("\t"); Serial.print("bias - "); Serial.println(_bias);
    Serial.print("\t"); Serial.print("Total - "); Serial.println(_totalError);
}
