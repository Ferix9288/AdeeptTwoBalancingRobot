/*

Header for Kalman Filter for sensor fusion of angle. 

Author: Felix Li 
Last Updated: 9/15/2018
*/

#ifndef AngleKalmanFilter_h
#define AngleKalmanFilter_h

/////////////////////////////
// Macros
/////////////////////////////
// #define DEBUG_ANGLEKF  //Enable debugging statements

//Kalman Filter
class AngleKalmanFilter
{
  public:

    /** Methods **/
    AngleKalmanFilter() {};
    AngleKalmanFilter(float angle, float QAngle, float QBias, float RAccel);
    float iterate(float dt, float rateGyro, float angleAccel);

  private:
    /** Variables **/ 
    float _angle; //state variable 
    float _angleDotBias;  //state variable  
    float _P[2][2] = { {0.0f, 0.0f}, {0.0f, 0.0f}}; //Covariance matrix of state variables
    float _QAngle; //Noise variance for angle
    float _QBias; //Noise variance for bias 
    float _RAccel; //Noise variance for accelerometer angle

    /** Methods **/
    void _predict(float dt, float rateGyro);
    void _update(float angleAccel);

    #ifdef DEBUG_ANGLEKF
    void _printMe();
    #endif
};

#endif

