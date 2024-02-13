#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import time
import brickpi3
import random
import math

BP = brickpi3.BrickPi3()

motor_left = BP.PORT_D
motor_right = BP.PORT_A

BP.set_motor_position_kp(motor_left, 30)
BP.set_motor_position_kd(motor_left, 150)

BP.set_motor_position_kp(motor_right, 30)
BP.set_motor_position_kd(motor_right, 150)

NUMBER_OF_PARTICLES = 100

particles = [(0, 680, 0) for _ in range(NUMBER_OF_PARTICLES)]
weights = [1 / NUMBER_OF_PARTICLES] * NUMBER_OF_PARTICLES

def get_pos():
    return (BP.get_motor_encoder(motor_left) + BP.get_motor_encoder(motor_right)) / 2


try:
    try:    
        # Reset encoder values for motors A and D
        BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
        BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right)) 
    except IOError as error:
        print(error)

    BP.set_motor_limits(motor_left, 50, 200)
    BP.set_motor_limits(motor_right, 50, 200)

    forward_distance = 680
    turning_distance = 172

    D = forward_distance / 4
    
    x0 = 0
    y0 = forward_distance
    x1, y1 = x0, y0

    for i in range(4): # Repeat 4 times for each side of the square
        # Reset encoder values for motors A and D
        BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
        BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right))
                
        for _ in range(4):  
            # Move forwards
            target = get_pos() + D
            BP.set_motor_position(motor_left, target)
            BP.set_motor_position(motor_right, target)

            motor_left_error = abs(BP.get_motor_encoder(motor_left) - target)
            motor_right_error = abs(BP.get_motor_encoder(motor_right) - target)

            while motor_left_error > 5 or motor_right_error > 5:
                time.sleep(0.01)
                #print(f"Left: {BP.get_motor_encoder(motor_left)}, Right: {BP.get_motor_encoder(motor_right)}")
                motor_left_error = abs(BP.get_motor_encoder(motor_left) - target)
                motor_right_error = abs(BP.get_motor_encoder(motor_right) - target)
 
            time.sleep(0.5)

            for j in range(NUMBER_OF_PARTICLES):
                e = random.gauss(0, 0.8) # forward spread
                f = random.gauss(0, 0.02)  # side spread

                particle_x_new = particles[j][0] + (D + e) * math.cos(particles[j][2])

                particle_y_new = particles[j][1] + (D + e) * math.sin(particles[j][2])
                theta_new = particles[j][2] + f

                particles[j] = (particle_x_new, particle_y_new, theta_new)
            
            print("drawParticles:" + str(particles))

        if i == 0:
            x1 += get_pos()
            y1 = y0
        elif i == 1:
            x1 = x0
            y1 -= get_pos()
        elif i == 2: 
            x1 -= get_pos()
            y1 = y0
        else: 
            x1 = x0
            y1 += get_pos()
            
        line = (x0, y0, x1, y1)

        print ("drawLine:" + str(line))

        x0, y0 = x1, y1
        
        # Turn left on the spot
        BP.set_motor_position(motor_left, BP.get_motor_encoder(motor_left) - turning_distance)
        BP.set_motor_position(motor_right, BP.get_motor_encoder(motor_right) + turning_distance)

        for j in range(NUMBER_OF_PARTICLES):
            g = random.gauss(0, 0.01) # rotational spread

            particle_x_new = particles[j][0]
            particle_y_new = particles[j][1]
            theta_new = particles[j][2] - math.pi / 2 + g

            particles[j] = (particle_x_new, particle_y_new, theta_new)
            
        print("drawParticles:" + str(particles))

        time.sleep(2)  # Depends on speed. May need to change after testing
    

except KeyboardInterrupt:
    BP.reset_all()

BP.reset_all()