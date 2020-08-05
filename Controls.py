'''
Module to control steering via PID Controller
Feedback will be the centre of the frame from the camera
Command will be the centre of the lane from the perception module

Steering Input to Arduino: 
75-80 = Centre, 180 = Full Right Lock, 0 = Full Left Lock

'''

import time
import main

#PID Parameters
Kp = 0.83
Ki = 0.1
Kd = 1.145
PID_Command = 0
sum_Error = 0
prev_Error = 0
prev_output = 0

def PID_SteeringControl(sys_time, Command, Feedback, sum_hist):
    global prev_output 
    #Check if variables have been defined
    try: prev_time
    except NameError: prev_time = sys_time

    try: sum_Error
    except NameError: sum_Error = 0

    try: prev_Error
    except NameError: prev_Error = 0

    #try: prev_output
    #except NameError: prev_output = 50

    #Culcaulting Time
    current_time = time.time()
    dt = current_time - prev_time
    
    #Calculating Error
    Error = -(Feedback - Command)
    sum_Error += (Error * dt)
    dError = (Error - prev_Error) / dt

    output = Kp * Error + Kd * dError #+ Ki * sum_Error

    prev_Error = Error
    prev_time = current_time

    #Limit Steering Output
    if (output > 100):
        output = 100
    elif (output < -100):
        output = -100
    '''
    #print("histogram values: ", sum_hist)
    if(sum_hist < 100000 and abs(output) > 5):
        output = prev_output
        print("Out of range, using last steering, ", output)
    '''   

    
    Steering = map_value(output, lower_in=-100, upper_in=100)

    print("PID Output: ", output, "Error: ", Error, "dError: ", dError, "Sum Error: ", sum_Error)
    prev_output = output
    return Steering #Map Output to Left and Right in Arduino Positive 100 = Full Left, Negative 100 = Full Right


def Throttle_Control(SteeringCommand):
    #full lock
    if (SteeringCommand >= 0 and SteeringCommand < 40) or (SteeringCommand > 140 and SteeringCommand <= 180):
        Throttle = 111
    #mid lock
    elif (SteeringCommand >= 40 and SteeringCommand < 75) or (SteeringCommand > 105 and SteeringCommand <= 140):
        Throttle = 111
    #straight
    elif (SteeringCommand >= 75 and SteeringCommand < 90) or (SteeringCommand >= 90 and SteeringCommand <= 105):
        Throttle = 111
    else:
        Throttle = 100
        print("Unknown Steering Command")

    return Throttle

#Mapping value from 0 to 180 for servo.h library in arduino to generate pwm signal
def map_value(value, lower_in, upper_in, lower_out=0, upper_out=180):
    return ((value - lower_in) * (upper_out - lower_out) // (upper_in - lower_in) + lower_out)
