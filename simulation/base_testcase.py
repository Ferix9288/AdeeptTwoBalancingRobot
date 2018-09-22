"""

Base Testcase. 

Useful for any utility function shared among different testcases.
"""

###########################
#Libraries / Imports
###########################
import unittest

###########################
#Helper Functions
###########################
"""Return True if two values are close in numeric value
    By default close is withing 1*10^-5 of each other
    i.e. 0.00001

    Resource: 
    https://stackoverflow.com/questions/12136762/assertalmostequal-in-python-unit-test-for-collections-of-floats
"""
def is_almost_equal(x,y, epsilon=1*10**(-5)):
    return abs(x-y) <= epsilon

###########################
#Main Class
###########################
class BaseTestcases(unittest.TestCase):

    """
        Helper function to do quick comparison between floats and print error message if not equal. 

        :param what: What variable we're comparing.
        :param value: What the function returned.
        :param expected: Expected value. 
        :return: None. Will throw exception if testcase fails. 
    """
    def checkFloatExpected(self, what, value, expected):
        self.assertTrue(is_almost_equal(value, expected), "Mismatch %s. %g != %g (expected). Note may be an issue with floating accuracy." % (what, value, expected))

