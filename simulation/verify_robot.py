"""

Quick unit testcases to verify Robot simulator.

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
from simulation import TwoWheelRobot, MAX_PITCH
import unittest

###########################
#Global Variables
###########################
#Toggles information about the testcase when running
VERBOSE = False 

###########################
#Helper Functions
###########################

"""
TO DO: To implement visualizer.
"""
class RobotVisualizer(object): 
    def __init__(self):
        pass 

    def visualize(self):
        pass

###########################
#Main UnitTestCase Class
###########################
class RobotTestcases(BaseTestcases):

    #Flag for visualization. Can be set via command line
    VIZ = False 

    def setUp(self):
        if RobotTestcases.VIZ: 
            self.viz = RobotVisualizer()

    def tearDown(self):
        pass

    """
        Based on series of PWM, expect system to behave a certain way. 
        
        :param initial_pitch: the initial condition.
        :param dt: change in time per iteration.
        :param pwm_list: sequence of PWM input.
        :param expected: expected pitch state as a result of pwm. 
        :return: None. Will throw exception if testcase fails. 
    """
    def _run_test(self, initial_pitch, dt, pwm_list, expected):


        if VERBOSE: 
            print "-" * 20
            print "Running Testcase %s" % (self.id())
            print 
            print "PWM Inputs :"
            for p in pwm_list:
                print "\t%g" % p

            print "Expected :"
            for e in expected:
                print "\t%g" % e

        #Visualize if flag true
        if RobotTestcases.VIZ: 
            self.viz.visualize()

        #Create Robot Object
        robot = TwoWheelRobot(initial_pitch)

        #iterate through 
        for i in range(len(pwm_list)): 
            pwm = pwm_list[i]
            exp = expected[i]

            pitch = robot.iterate(pwm, dt)
            self.checkFloatExpected("Pitch at iteration %d" % i, pitch, exp)

    #----------------------------------------
    #--- Testcases ---
    #-------------------------------------------

    """ Handle where only gravity is the real input. Fall forward. """
    def testcase_only_gravity(self):

        pwm_input = [
            0,
            0, 
            0,
            0,
            0
        ]

        expected = [
            11.5336983812,
            42.3819574328,
            MAX_PITCH,
            MAX_PITCH,
            MAX_PITCH
        ]

        self._run_test(0.0001, 0.5, pwm_input, expected)


    """ Handle where only gravity is the real input. Fall negative. """
    def testcase_only_gravity2(self):

        pwm_input = [
            0,
            0, 
            0,
            0,
            0
        ]

        expected = [
            -11.5336983812,
            -42.3819574328,
            -MAX_PITCH,
            -MAX_PITCH,
            -MAX_PITCH
        ]

        self._run_test(-0.0001, 0.5, pwm_input, expected)

  


if __name__ == '__main__':
    if '--viz' in sys.argv:
        RobotTestcases.VIZ = True
        sys.argv.pop()
    unittest.main()
