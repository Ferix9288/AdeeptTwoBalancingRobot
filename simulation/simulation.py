"""

Simulate the two wheel balance robot. 
Note: equations taken from modeling.xlsx

Author: Felix Li 
Date: 9/22/2018
"""

###########################
#Libraries / Imports
###########################
import math 
import random 

###########################
#Global Variables
###########################
MAX_PITCH = 60 #robot can't fall more than this angle 

###########################
#Two Wheel Robot Class
###########################
class TwoWheelRobot(object): 

    """ Initialize starting pitch. """    
    def __init__(self, initial_pitch): 
        self.pitch = initial_pitch

    """
    Adjust pitch based on motor input. 
    :param: pwm. The PWM input into the motor.
    :param: dt. Elasped time. 
    :return: new state.
    """
    def iterate(self, pwm, dt):
        gravity_rate = self._gravity(self.pitch)
        motor_rate = self._motor(pwm) * -1 # + PWM, negative rotation; - pwm, + rotation
        total_rate = motor_rate + gravity_rate
        new_pitch = self.pitch + total_rate*dt 
        self.pitch = new_pitch

        #constrain to MAX_PITCH. Otherwise, modeling functions would blow up due to exponential. 
        self.pitch = max(self.pitch, -MAX_PITCH)
        self.pitch = min(self.pitch, MAX_PITCH)
        return self.pitch
        
    """
    Based on current pitch, return pitch rotation rate due to gravity.  
    Equation: 
        y = 23.067e^(0.0853x)
    :return: rate in deg/s 
    """
    def _gravity(self, pitch): 
        sign = 1 #default positive. Fall forward = +, fall backward = -/
        if pitch > 0: 
            sign = 1
        elif pitch < 0: 
            sign = -1
        else: #pitch = 0. Toss up if falling forward or backward
            sign = random.choice([-1, 1])
        return sign * 23.067 * math.exp(0.0853*pitch*sign)

    """
    Based on PWM, return rotation rate effect from motor.
    Equation: 
        y = 103.5ln(x) - 355.14
        Note: PWM constrained to 0 if negative in y direction. 

    :return: rate in deg/s
    """
    def _motor(self, pwm):
        #Arduino only accepts duty cycle of int 
        pwm = int(pwm)
        if pwm == 0: 
            return 0 
        else: 
            sign = 1 
            if pwm > 0: 
                sign = 1 
            else:
                sign = -1 
                pwm *= -1 #make it back to positive, but multiply negation 
            value = 103.5 * math.log(pwm) - 355.14 
            #if function is below 0 (i.e. from 0 to 30, then equate it to just 0)
            if (value < 0): 
                value = 0
            return value * sign



###########################
#Class to replicate Motor Control PID
###########################
#For simplicity sake, not splitting into left vs right motor.
class MotorPID(object): 

    """
    Initializer.
    :param: ref. Reference point. 
    :param: kp. P constant. 
    :param: ki. I constant. 
    :param: kd. D constant. 
    """    
    def __init__(self, ref, kp, ki, kd, max_pwm = 255): 
        self.ref = ref 
        self.kp = kp 
        self.ki = ki 
        self.kd = kd
        self.max_pwm = max_pwm

        self._prev_error = 0
        self._total_error = 0

    """
    Iteration of the PID controller. 
    :param: pitch. Current pitch of system. 
    :param: dt. Time elapsed. 

    """
    def iterate(self, pitch, dt): 
        cur_error = self.ref - pitch 
        
        #p term 
        p_term = self.kp * cur_error 

        #i term 
        self._total_error += cur_error
        i_term = self.ki * self._total_error*dt

        #d term 
        delta_error = (cur_error - self._prev_error)/dt 
        d_term = self.kd * delta_error

        pid = p_term + i_term + d_term

        #for next iter 
        self._prev_error = cur_error

        #constrain to -MAX_PWM to MAX_PWM
        pid = max(pid, -self.max_pwm)
        pid = min(pid, self.max_pwm)

        #because you want motor to produce PWM where robot falls, requires negation. Sign of PWM handled directionality of motors
        pid *= -1 
        return pid