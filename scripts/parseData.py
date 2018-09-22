"""

Create a python script that parses data output from Bluetooth. The processed data is then used 
in modeling.xlsx to get a rough equation.

""" 


import argparse
import re

####################
# Global Variables
####################

#::Regex / Pattern matching::
KALMAN_IN_REGEX = r'.*?Measurement:\s*(\-?\d+\.\d+); rateGyro:\s*(\-?\d+\.\d+); dt:\s*(\-?\d+\.\d+)$'
PITCH_REGEX = r'.*?Pitch:(\-?\d+\.\d+)$'
PITCH2_REGEX = r'.*?Filtered Pitch:(\-?\d+\.\d+)$'
PID_REG = r'.*?PID: (\-?\d+\.\d+) (\-?\d+\.\d+)' 

"""
Parse file to collect how pitch changes with respect to gravity & motor pwm.
Note: for gravity files, ignore PWM output (motor output fixed to 0)

:param file_name: Name of file with data dump. 
"""
def parse_file(file_name, only_gravity = False): 
    f = open(file_name, 'r')

    measure = None 
    rate_gyro = None 
    dt = None 
    initial_pitch = None 
    next_pitch = None
    left_motor = None 
    right_motor = None 


    print "--Reading File %s--" % file_name
    lines = f.readlines()
    last_line = lines[-1].strip()
    for l in lines:
        l = l.strip()
        if l == last_line or re.search("Iteration", l): #marks new iteration 
            if measure: #if defined 
                rate = (next_pitch - initial_pitch)/dt 
                if only_gravity:
                    print "%g %g" % (initial_pitch, rate) #print out pitch to rate of change
                else: 
                    #for pwm, take average of the two motors 
                    pwm = (left_motor + right_motor)/2.0
                    #both motor and gravity in effect.
                    print "%g %g %g" % (initial_pitch, rate, pwm)

        kalman_in = re.match(KALMAN_IN_REGEX, l)
        if kalman_in: 

            measure, rate_gyro, dt = kalman_in.groups()

            measure = float(measure)
            rate_gyro = float(rate_gyro)
            dt = float(dt)
            continue


        pitch2 = re.match(PITCH2_REGEX, l)
        if pitch2: 
            next_pitch = pitch2.group(1) 
            next_pitch = float(next_pitch)
            continue 

        pitch = re.match(PITCH_REGEX, l)
        if pitch:
            initial_pitch = pitch.group(1)
            initial_pitch = float(initial_pitch)
            continue

        pid = re.match(PID_REG, l)
        if pid: 
            left_motor, right_motor = pid.groups() 
            left_motor = float(left_motor)
            right_motor = float(right_motor)

if __name__ == "__main__": 

    parser = argparse.ArgumentParser()
    parser.add_argument("listFile", metavar="listFile", help = "Text file that list out all other text files to grab.")
    parser.add_argument("-v", "--verbosity", type=int, help="increase output verbosity")

    args = parser.parse_args()
    list_file_name = args.listFile

    list_file = open(list_file_name, 'r')
    for l in list_file.readlines(): 
        l = l.strip() #strip newline 

        #see if file is of 'gravity' type
        only_gravity = False
        if re.search('gravity', l):
            only_gravity = True 
        parse_file(l, only_gravity)



