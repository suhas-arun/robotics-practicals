#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import time
import brickpi3

BP = brickpi3.BrickPi3()

try:
    try:    
        # Reset encoder values for motors A and D
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) 
    except IOError as error:
        print(error)

    # Motor A is for moving forward
    # Motor D is for turning left on the spot
    BP.set_motor_limits(BP.PORT_A, 50, 200) 
    BP.set_motor_power(BP.PORT_D, BP.MOTOR_FLOAT) 

    forward_distance_degrees = 360  # May need to changed based on calibration (want it to be 40cm)
    turn_angle_degrees = 90  # May need to change based on calibration (want it to make sharp turns)

    for _ in range(4):  # Repeat 4 times for each side of the square
        BP.set_motor_position(BP.PORT_A, forward_distance_degrees)
        time.sleep(1)  # Depends on speed. May need to change after testing

        # Turn left on the spot
        BP.set_motor_position(BP.PORT_D, -turn_angle_degrees)  
        time.sleep(1)

except KeyboardInterrupt:
    BP.reset_all()
