#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import time
import brickpi3

BP = brickpi3.BrickPi3()

motor_left = BP.PORT_D
motor_right = BP.PORT_A

BP.set_motor_position_kp(motor_left, 30)
BP.set_motor_position_kd(motor_left, 150)

BP.set_motor_position_kp(motor_right, 30)
BP.set_motor_position_kd(motor_right, 150)


try:
    try:    
        # Reset encoder values for motors A and D
        BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
        BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right)) 
    except IOError as error:
        print(error)

    BP.set_motor_limits(motor_left, 50, 200)
    BP.set_motor_limits(motor_right, 50, 200)

    forward_distance = 693
    turning_distance = 190

    for _ in range(4):  # Repeat 4 times for each side of the square
        # Reset encoder values for motors A and D
        BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
        BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right))
        
        # Move forwards
        BP.set_motor_position(motor_left, forward_distance)
        BP.set_motor_position(motor_right, forward_distance)
        
        motor_left_error = abs(BP.get_motor_encoder(motor_left) - forward_distance)
        motor_right_error = abs(BP.get_motor_encoder(motor_right) - forward_distance)

        
        while motor_left_error > 5 or motor_right_error > 5:
           time.sleep(0.01)
           print(f"Left: {BP.get_motor_encoder(motor_left)}, Right: {BP.get_motor_encoder(motor_right)}")
           motor_left_error = abs(BP.get_motor_encoder(motor_left) - forward_distance)
           motor_right_error = abs(BP.get_motor_encoder(motor_right) - forward_distance)

        
        print(f"Motor A position: {BP.get_motor_encoder(BP.PORT_A)}, Motor D position: {BP.get_motor_encoder(BP.PORT_D)}")

        
        # Reset encoder values for motors A and D
        BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
        BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right)) 

        # Turn left on the spot
        BP.set_motor_position(motor_left, -turning_distance)
        BP.set_motor_position(motor_right, turning_distance)
        
        time.sleep(2)  # Depends on speed. May need to change after testing


except KeyboardInterrupt:
    BP.reset_all()

BP.reset_all()
