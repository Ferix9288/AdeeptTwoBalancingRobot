
## Description 

The following side project is to write my own Kalman Filter and PID controller to balance a two wheel robot. 

**High Overview**
1. Use Kalman filter for sensor fusion of both accelerometer and gyroscope measurements to calculate pitch. In the end, we will get a better estimate of true angle by accounting for short-term noises (the accelerometer) and long-term biases (drift from gyroscope)
2. Use this estimated angle in a PID controller and aim to get this pitch angle 0. 
3. Simulated and modeled the environment via data collection. Then use this to apply an optimization algorithm (hill climbing) to fine-tune PID constants.

**Hardware**

Used Adeept Arduino Two Wheel Balancing Robot kit: [Amazon Link](https://www.amazon.com/Adeept-Self-Balancing-Accelerometer-Gyroscope-Avoidance/dp/B076XVD9B9)

**Software**
    
    Main
        selfBalance.ino //main code 

    Libraries
        PIDController //my library to import in Arduino   
        AngleKalmanFilter //my library to import in Arduino 
        I2CDev1 //third party     
        MPU6050 //third party
        MsTimer2 //third party 

Third party libraries can be found here: 
https://www.adrive.com/public/97GXSs/Adeept_SelfBalancingRobotKit-V1.0.zip

## Modeling
Using bluetooth, dumped pitch data and timestamps to model the real system in a simulated environment. The primary motivator is so that I can then code an optimization algorithm to explore the PID constant space. 

**Specific Steps/Overview**
1. Collect data without any motor control used on the robot. This is to model the effects of gravity on pitch on the robot.   
2. Collect data varying the PWM of the robot to the motor controller. 
3. With the data from #2, subtract away the model of gravity to get a rough estimate of how motors affect pitch. 
4. Re-code PID algorithm and verify correctness/equivalency in Python. 
5. With simulated environment, iterate using hill-climbing to explore PID space. 

## Current Status
Kalman Filter on pitch successfully implemented and tested. 

Having trouble with setting the PID constants of the controller where it can only stabilize for about 7 to 8 seconds. I'm able to dynamically change and fine-tune the constants via Bluetooth, but have yet to find the magical combination of numbers. This is even after simulating the environment as well. 


## Future steps
1. Continue fine-tuning PID constants. Once done, record my baby in action.
    - Will try doing a bluetooth dump on PID term contributions (the P, I, and D)
    - Maybe cry some more as to how painful this process is. 
    - Also delve more into Control theory. 
2. Handle movement control. (i.e. turning, forward, reverse)
3. Object avoidance. 
4. Follow me. 

## References, Appendix, and More Info

**Main Sources of Inspiration**
1. [Adeept's Provided Code](https://www.adrive.com/public/97GXSs)
2. [Lauszus Balancing Robot](https://github.com/TKJElectronics/BalancingRobotArduino)

**More FYI**

Please refer to [notes](NOTES). These are my own personal notes whilst working on this side project. For example, it contains my write-up of the theory behind Kalman Filter, how to handle raw data from sensors, tuning PID constants, etc. 
