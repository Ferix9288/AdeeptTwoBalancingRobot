"""

Script to run a hill climbing algorithm (twiddle) to find best PID constants.

Author: Felix Li
"""

###########################
#Libraries / Imports
###########################
from simulation import TwoWheelRobot, MotorPID, MAX_PITCH

###########################
#Global Variables
###########################
REFERENCE = 0 #for now, set to 0 pitch angle as reference 
SUCCESS = 60 #success if can stabilize robot for 60 seconds, i.e. one minute 
SAMPLING_RATE = 0.05 #sampling rate of actual car is 50ms 
FAIL_SCORE = 9999 #score if testcase fails 
DEBUG = False #debug statements 

###########################
#Helper Functions 
###########################

"""
Runs a simulation with designated p, i, and d constants. 

:param p: kp 
:param i: ki 
:param d: kd 
"""
def run_simulation(p, i, d): 

    #robot initially at pitch 0
    robot = TwoWheelRobot(0)
    pid = MotorPID(REFERENCE, p, i, d)

    #initial pwm 
    pwm = pid.iterate(0, SAMPLING_RATE)

    time_elapsed = 0
    total_error = 0 
    success = True 

    #mainly for debugging 
    states = []
    inputs = []
    while (time_elapsed <= 60): 
        pitch = robot.iterate(pwm, SAMPLING_RATE)
        pwm = pid.iterate(pitch, SAMPLING_RATE)

        states.append(pitch)
        inputs.append(pwm)
        
        if pitch == MAX_PITCH or pitch == -MAX_PITCH:  #robot fell.
            success = False 
            break

        total_error += abs(pitch - REFERENCE)
        time_elapsed += SAMPLING_RATE

    if DEBUG:
        print "State / Inputs"
        for i in range(len(states)): 
            s = states[i]
            inp = inputs[i]
            print "\t%g deg %g pwm" % (s, inp)
    if not success:
        return FAIL_SCORE
    else:
        return total_error #lower the error, the better the score

"""
Twiddle algorithm from Robotics class. 
Essentially a hill climbing algorithm where constants are tweaked. 

:param p: starting PID parameters.
:param dp: how to vary those param.
:param tol: terminate algorithm when change in constants is less than tol(erance) 
"""
def twiddle(p, dp, tol=0.2): 

    # p = [10.0, 1.0, 0.1]
    # dp = [20.0, 5, 0.5]

    best_err = FAIL_SCORE

    # TODO: twiddle loop here
    dp_sum = sum(dp)
    while (dp_sum > tol):
        
        for i in range(len(p)):
            #try incrementing by probe step
            p[i] += dp[i]
            attempt1, new_error = try_new_pid(p, best_err)
            if attempt1: 
                best_err = new_error
                dp[i] *= 1.1 
            else: 
                #try decrementing by probe step
                p[i] -= 2 * dp[i]
                attempt2, new_error = try_new_pid(p, best_err)
                if attempt2: 
                    best_err = new_error
                    dp[i] *= 1.1
                else:
                    #revert p back to what it was
                    p[i] += dp[i] 
                    #smaller p range 
                    dp[i] *= 0.9

        dp_sum = sum(dp)
        # print dp_sum

    return p, best_err

"""
Quick helper function to twiddle algorithm.
:param p: current pid terms. 
:param best_error: best error seen thus far
:return bool, new_error: True if better error, false otherwise
"""
def try_new_pid(p, best_err):

    new_error = run_simulation(p[0], p[1], p[2])
    if DEBUG: 
        print "PID %s: %g " % (p, new_error)
    if new_error < best_err:
        return True, new_error
    else: 
        return False, new_error


#Finding Best P 
# p = [10.0, 0, 0]
# dp = [20.0, 0, 0]
#P = 42.75000772457 ; score = 832.527

#Now find I 
# p = [42.75, 1, 0]
# dp = [10.0, 1, 0]
# Best PID: [41.74999999999999, 1.3874204889999995, 0.0]
# Score: 1033.13

#Add D 
p = [41.74, 1.38, 0.05]
dp = [10, 1, 0.1]
# Best PID: [40.74000000000001, 0.15022240800490438, -0.05000000000000003]
# Score: 981.282


#Actually run twiddle 
p, best_err = twiddle(p, dp)
print "Best PID: %s" % p 
print "Score: %g" % best_err