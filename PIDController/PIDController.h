/*

Header for  PID Controller.

Author: Felix Li 
Last Updated: 9/15/2018
*/


#ifndef PIDController_h
#define PIDController_h

/////////////////////////////
// Macros
/////////////////////////////
// #define DEBUG_PID  //Enable debugging statements

//Kalman Filter
class PIDController
{
  public:

    /** Methods **/
    PIDController() {};
    PIDController(float ref, float kp, float kd, float ki, float constraint = 1000, float bias = 0) {
        _ref = ref; 
        _kp = kp; 
        _kd = kd; 
        _ki = ki;
        _constraint = constraint;
        _bias = bias;
        _prevError = 0;
        _totalError = 0;
    }

    float update(float measurement, float dt);
    void setKp(float value) {_kp = value;}
    void setKi(float value) {_ki = value;}
    void setKd(float value) {_kd = value;}
    void setBias(float bias) {_bias = bias;}

    //print outs private variables
    void printMe();

  private:
    /** Variables **/ 
    float _ref; //target reference point
    float _kp; //proportional gain  
    float _kd; //derivative gain
    float _ki; //integral gain 
    float _constraint; //constraint bounds on total error
    float _bias; //adds bias to the system. Needed to account for deadzones.

    float _prevError; //previous error
    float _totalError; //total error 
};

#endif
