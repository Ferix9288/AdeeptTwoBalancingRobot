/*
Kalman Filter implementation for sensor fusion (accelerometer and gyroscope) to calculate 
the pitch (arctan2 of y/z). 

See project's README for more details.

Author: Felix Li 
Last Updated: 9/15/2018
*/


/////////////////////////////
//  Header Files
/////////////////////////////
#include "AngleKalmanFilter.h" 

#ifdef DEBUG_ANGLEKF
#include "HardwareSerial.h" //for debugging
#endif 

/////////////////////////////
// Constructor 
/////////////////////////////

/**
    Constructor for said class. 

    State:
        x = [angle
             w_b] 
        where:
            angle is the current pitch estimate 
            w_b is the  angle bias/error accumulated from both the accelerometer and gyroscope.
    
    State Function: 
        F = [1 -dt 
             0  1]

    Control Function: 
        B = [dt 
             0]

    Control Input: 
        u = [w_gyro]

    P = Covariance matrix of the state angle w_b 
    Q = Covariance matrix of noise 
      =  [var_angle 0 
          0   var_bias]
          -But breaking it down to variables

    R = Covariance matrix of measurements


    @param angle. Initial state angle.
    @param QAngle. Noise variance of angle. 
    @param QBias. Noise variance of bias. 
    @param RAccel. Noise variance of accelerometer. 
    @return instance.
*/ 
AngleKalmanFilter::AngleKalmanFilter(float angle, float QAngle, float QBias, float RAccel)
{
    _angle = angle; 

    //assumption is that it starts at 0 bias, and thus covariance of P is all 0 as well.
    //https://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    _angleDotBias = 0.0f;
    //Done in header: _P = { {0.0f, 0.0f}, {0.0f, 0.0f}};
    _QAngle = QAngle;
    _QBias = QBias;
    _RAccel = RAccel;
}

/////////////////////////////
// Public Methods
/////////////////////////////

/**
    Core function of Kalman Filter. Iterates with a predict and update step. 

    @param dt. Time delta.
    @param rateGyro. Gyroscope measurement reading.
    @param angleAccel. Angle of pitch as calculated by the accelerometer. 
    @return new calculated angle from filter (float)
*/
float AngleKalmanFilter::iterate(float dt, float rateGyro, float angleAccel)
{
    _predict(dt, rateGyro); 

    #ifdef DEBUG_ANGLEKF
    Serial.println("AngleKalmanFilter::iterate. After predict.");
    _printMe();
    #endif

    _update(angleAccel);

    #ifdef DEBUG_ANGLEKF
    Serial.println("AngleKalmanFilter::iterate. After update.");
    _printMe();
    #endif

    return _angle;
}

/////////////////////////////
// Private Methods
/////////////////////////////
/**
    Predict step of Kalman Filter.
    
    Next State:
        x^ = Fx  * Bu + w 
        where:
            x^ = a priori state 
            F = State function 
            x = state 
            B = Control function
            u = control input 
            w = gaussian noise 
        *note: definitions explained in constructor
        
        -Final Equations-
        angle^ = angle - dt * w_b + dt * w_gyro + noise 
        w_b = w_b 

    Covariance: 
        P^ = F*P*F^T + Q 
        where: 
            P^ = priori covariance 
            P = current covariance 
            Q = Covariance of noise 

    @param dt. Time delta.
    @param rateGyro. Gyroscope measurement reading.
    @return None.
*/
void AngleKalmanFilter::_predict(float dt, float rateGyro)
{
    //for memory and legibility, just writing out the state update equations 
    _angle += dt*(rateGyro - _angleDotBias);
    //_angleDotBias = _angleDotBias  -- Bias stays the same 

    //update the Covariance matrix P 
    //Note: below is a reduction of the matrix multiply. See readme.txt for the breakdown.
    _P[0][0] += -_P[1][0] * dt - (_P[0][1] - _P[1][1]*dt)*dt + _QAngle*dt;
    _P[0][1] -= _P[1][1] * dt; 
    _P[1][0] -= _P[1][1] * dt;
    _P[1][1] += _QBias * dt;
}


/**
    Update step. 

    Error/Innovation = y =  Z - H*x^
        -where:
            Z is the measurement matrix
            H is the measurement function
            x^ is the priori state.

    Kalman Gain = K = P*H^T / S 
        where S = H*P*H^T + R 
        *Note: P is actually P^, i.e. new P from predicting

    New State = x =  x^ - K*y
    New Covariance = P = (I-KH)P^

    In case, 
        Z is just pitch angle from accelerometer. 
        H is a matrix of [1 0] to only return what we can observe, i.e. the angle in our state

    @param angleAccel. Angle of pitch as calculated by the accelerometer. 
    @return None.
*/
void AngleKalmanFilter::_update(float angleAccel)
{
    //Calculate error
    //because P is a 2x2 and H is 1x2, end result is a 1x1 and simplified result is below.
    float error = angleAccel - _angle; 

    //Calculate Kalman Gain
    //end result of H*P*H^T + R 
    float S = _P[0][0] + _RAccel;

    //K becomes a 2x1 matrix
    float K0 = _P[0][0]/S;
    float K1 = _P[1][0]/S; 

    //update state
    _angle += K0*error; 
    _angleDotBias += K1*error; 

    //update covariance matrix P of latest state
    //end result of (I-KH)*P^ (see readme)
    float P00Temp = _P[0][0];
    float P01Temp = _P[0][1];
    _P[0][0] -= K0*P00Temp;
    _P[0][1] -= K0*P01Temp; 
    _P[1][0] -= K1*P00Temp;
    _P[1][1] -= K1*P01Temp; 
}


#ifdef DEBUG_ANGLEKF
/**
    Helper function to print internal variables. 
*/
void AngleKalmanFilter::_printMe() 
{
    Serial.print("Angle:"); Serial.println(_angle);
    Serial.print("AngleDotBias:"); Serial.println(_angleDotBias);
    Serial.print("P:\t"); 
    Serial.print(_P[0][0]); Serial.print("\t"); Serial.println(_P[0][1]);
    Serial.print("\t"); Serial.print(_P[1][0]); Serial.print("\t"); Serial.println(_P[1][1]);

}
#endif 
