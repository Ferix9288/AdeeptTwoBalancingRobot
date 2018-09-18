"""
To not bog down code with large chunks of comment, I thus carried some of it over here. 

"""



Code Optimzation of calculating Error Matrix in predict: 
(Otherwise have to keep accessing a 2D array more times than necessary)
    //use temporary/intermediary matrix Pdot - specifically this is F*P
    this._PDot[0][0] = P[0][0] - P[1][0] * dt
    this._PDot[0][1] = P[0][1] - P[1][1] * dt 
    this._PDot[1][0] = P[1][0]
    this._PDot[1][1] = P[1][1]

    //now PDot * F^T + Q
    this._P[0][0] = this._PDot[0][0] - dt * this._PDot[0][1] + Q[0][0]
    this._P[0][1] = this._PDot[0][1] + Q[0][1]
    this._P[1][0] = this._PDot[1][0] - dt* this._PDot[1][1] + Q[1][0]
    this._P[1][1] = this._PDot[1][1] + Q[1][1]


    The above is equivalent to:  Q[0][0] = Qangle, Q[1][1] = Qbias
    this->_P[0][0] -= P[1][0] * dt - (P[0][1] - P[1][1]*dt)*dt + this->_QAngle*dt;
    this->_P[0][1] -= this->_P[1][1] * dt; 
    this->_P[1][0] -= this->_P[1][1] * dt;
    this->_P[1][1] += this->_QBias * dt;


Calculation for Error Matrix in update: 
    P = (I-KX) * P^

    ([1 0  - [K0  [1 0] ) * P 
      0 1]    K1]

      -Distribute out P 
      -K*H becomces [ [K0 0] [K1 0]]

    -[ [P0 P1] [P2 P3] ] - [ [K0*P0 K0*P1] [K1*P0 K1*P1]

    this->_P[0][0] -= K0*P0Temp;
    this->_P[0][1] -= K0*P1Temp; 
    this->_P[1][0] -= K1*P0Temp;
    this->_P[1][1] -= K1*P1Temp; 


Below Code directly modeled after https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp
    -My code wasn't working and hence resorted to this tactic 
    -This code worked beautifully 
    -Culprit was this line: 
            this->_P[0][0] -= this->_P[1][0] * dt - (this->_P[0][1] - this->_P[1][1]*dt)*dt + this->_QAngle*dt;
            
            converts to P[0][0] - P[1][0]*dt - _P[0][1]*dt + P[1][1]dt^2 + Qangle * dt

        Apparently the -= operator means -= (the entire expression!!!!).
        ~LESSON LEARNED~


// The this->_angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float AngleKalmanFilter::iterate(float dt, float rateGyro, float angleAccel) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    float rate = rateGyro - this->_angleDotBias;
    this->_angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    this->_P[0][0] += dt * (dt*this->_P[1][1] - this->_P[0][1] - this->_P[1][0] + this->_QAngle);
    this->_P[0][1] -= dt * this->_P[1][1];
    this->_P[1][0] -= dt * this->_P[1][1];
    this->_P[1][1] += this->_QBias * dt;

    #ifdef DEBUG_ANGLEKF
    Serial.println("AngleKalmanFilter::iterate. After predict.");
    this->_printMe();
    #endif
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = this->_P[0][0] + this->_RAccel; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = this->_P[0][0] / S;
    K[1] = this->_P[1][0] / S;

    // Calculate this->_angle and this->_angleDotBias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = angleAccel - this->_angle; // Angle difference
    /* Step 6 */
    this->_angle += K[0] * y;
    this->_angleDotBias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = this->_P[0][0];
    float P01_temp = this->_P[0][1];

    this->_P[0][0] -= K[0] * P00_temp;
    this->_P[0][1] -= K[0] * P01_temp;
    this->_P[1][0] -= K[1] * P00_temp;
    this->_P[1][1] -= K[1] * P01_temp;

    #ifdef DEBUG_ANGLEKF
    Serial.println("AngleKalmanFilter::iterate. After update.");
    this->_printMe();
    #endif

    return this->_angle;
};
