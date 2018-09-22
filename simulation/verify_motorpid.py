"""

Quick unit testcases to verify the Motor PID. 
Cross checked with the bluetooth output of the car.

Author: Felix Li 
"""

###########################
#Libraries / Imports
###########################
import sys 
import numpy as np
import matplotlib.pyplot as plt
import math
import random 
from base_testcase import BaseTestcases
from simulation import MotorPID
import unittest

###########################
#Global Variables
###########################
#Toggles information about the testcase when running
VERBOSE = False 

#Main K, I, D terms for testing. 
TEST_KP = 10 
TEST_KI = 1 
TEST_KD = 0.1

###########################
#Helper Functions
###########################

"""
TO DO: To implement visualizer.
"""
class MotorPIDVisualizer(object): 
    def __init__(self):
        pass 

    def visualize(self):
        pass

###########################
#Main UnitTestCase Class
###########################
class MotorPIDTestcases(BaseTestcases):

    #Flag for visualization. Can be set via command line
    VIZ = False 

    def setUp(self):
        if MotorPIDTestcases.VIZ: 
            self.viz = MotorPIDVisualizer()

    def tearDown(self):
        pass

    """
        Based on input stream of pitches, see how motor PID behaves.
            
        :param controls: what parameters of PID for controller.
        :param pitch_list: State of the robot.
        :param dt: change in time per iteration.
        :param expected: expected PWM output.
        :return: None. Will throw exception if testcase fails. 
    """
    def _run_test(self, controls, pitch_list, dt, expected):

        if VERBOSE: 
            print "-" * 20
            print "Running Testcase %s" % (self.id())
            print 
            print "Pitch States :"
            for p in pitch_list:
                print "\t%g" % p

            print "Expected :"
            for e in expected:
                print "\t%g" % e

        #Visualize if flag true
        if MotorPIDTestcases.VIZ: 
            self.viz.visualize()

        #Create MotorPID Object
        ref, kp, ki, kd = controls 
        pid = MotorPID(ref, kp, ki, kd) 

        #iterate through 
        for i in range(len(pitch_list)): 
            pitch = pitch_list[i]
            exp = expected[i]

            pwm = pid.iterate(pitch, dt)
            self.checkFloatExpected("PWM at iteration %d" % i, pwm, exp)

    #----------------------------------------
    #--- Testcases ---
    #-------------------------------------------

    """ Handle with only p term/. """
    def testcase_p(self):

        #ref, kp, ki, kd
        controls = [
            0, TEST_KP, 0, 0
        ]

        pitch_list = [
            10,
            20, 
            30,
            -10,
            -20,
            -30
        ]

        expected = [
            100,
            200,
            255,
            -100,
            -200,
            -255 
        ]

        self._run_test(controls, pitch_list, 1, expected)

    """ Handle with only i term. """
    def testcase_i(self):

        #ref, kp, ki, kd
        controls = [
            0, 0, TEST_KI, 0
        ]

        pitch_list = [
            10,
            10,
            10,
            -10,
            -10,
            -10
        ]

        expected = [
            5,
            10,
            15,
            10,
            5,
            0
        ]

        self._run_test(controls, pitch_list, 0.5, expected)

    """ Handle with only i term. """
    def testcase_d(self):

        #ref, kp, ki, kd
        controls = [
            0, 0, 0, TEST_KD
        ]

        pitch_list = [
            0,
            5,
            15,
            0,
            -5,
            -15
        ]

        expected = [
            0,
            1,
            2,
            -3,
            -1,
            -2
        ]

        self._run_test(controls, pitch_list, 0.5, expected)


    """ These are sampled from car via bluetooth. Changes if c++ code is different. """
    # def testcase_cpp(self):


    #     #ref, kp, ki, kd
    #     controls = [
    #         0, TEST_KP, TEST_KI, TEST_KD
    #     ]

    #     pitch_list = [
    #         0,
    #         5,
    #         15,
    #         0,
    #         -5,
    #         -15
    #     ]

    #     expected = [
    #         0,
    #         1,
    #         2,
    #         -3,
    #         -1,
    #         -2
    #     ]
    #     self._run_test(controls, pitch_list, 0.5, expected)



if __name__ == '__main__':
    if '--viz' in sys.argv:
        MotorPIDTestcases.VIZ = True
        sys.argv.pop()
    unittest.main()
