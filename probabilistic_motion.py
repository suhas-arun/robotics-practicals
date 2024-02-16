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

TURNING_FACTOR = 117 # How much to turn to rotate 1 radian
MOVING_FACTOR = 17 # How much to move forward 1 cm

particles = [(0, 680, 0) for _ in range(NUMBER_OF_PARTICLES)]
weights = [1 / NUMBER_OF_PARTICLES] * NUMBER_OF_PARTICLES

def get_left_pos():
    return BP.get_motor_encoder(motor_left)
    
def get_right_pos():
    return BP.get_motor_encoder(motor_right)

def forward(particles, distance):
    target_left = get_left_pos() + distance
    target_right = get_right_pos() + distance
    BP.set_motor_position(motor_left, target_left)
    BP.set_motor_position(motor_right, target_right)

    motor_left_error = abs(BP.get_motor_encoder(motor_left) - target_left)
    motor_right_error = abs(BP.get_motor_encoder(motor_right) - target_left)

    while motor_left_error > 5 or motor_right_error > 5:
        time.sleep(0.01)
        motor_left_error = abs(BP.get_motor_encoder(motor_left) - target_left)
        motor_right_error = abs(BP.get_motor_encoder(motor_right) - target_right)

    time.sleep(0.5)

    return update_particles_after_forward(particles, distance)


def update_particles_after_forward(particles, D):
    for j in range(NUMBER_OF_PARTICLES):
        e = random.gauss(0, 0.8) # forward spread
        f = random.gauss(0, 0.02)  # side spread

        particle_x_new = particles[j][0] + (D + e) * math.cos(particles[j][2])

        particle_y_new = particles[j][1] + (D + e) * math.sin(particles[j][2])
        theta_new = particles[j][2] + f

        particles[j] = (particle_x_new, particle_y_new, theta_new)

    return particles

def rotate(particles, angle):
    turning_distance = TURNING_FACTOR * angle
    #print("turning", turning_distance)
    BP.set_motor_position(motor_left, BP.get_motor_encoder(motor_left) - turning_distance)
    BP.set_motor_position(motor_right, BP.get_motor_encoder(motor_right) + turning_distance)
    return update_particles_after_rotation(particles, angle)


def update_particles_after_rotation(particles, angle):
    for j in range(NUMBER_OF_PARTICLES):
        g = random.gauss(0, 0.01) # rotational spread

        particle_x_new = particles[j][0]
        particle_y_new = particles[j][1]
        theta_new = particles[j][2] - angle + g

        particles[j] = (particle_x_new, particle_y_new, theta_new)

    return particles

if __name__ == "__main__":
    try:
        try:    
            # Reset encoder values for motors A and D
            BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
            BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right)) 
        except IOError as error:
            print(error)

        BP.set_motor_limits(motor_left, 50, 200)
        BP.set_motor_limits(motor_right, 50, 200)

        x0 = 0
        y0 = 40 * MOVING_FACTOR
        x1, y1 = x0, y0

        for i in range(4): # Repeat 4 times for each side of the square
            # Reset encoder values for motors A and D
            BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
            BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right))
                    
            for _ in range(4):
                particles = forward(particles, 10 * MOVING_FACTOR)
                print("drawParticles:" + str(particles))

            if i == 0:
                x1 += get_left_pos()
                y1 = y0
            elif i == 1:
                x1 = x0
                y1 -= get_left_pos()
            elif i == 2: 
                x1 -= get_left_pos()
                y1 = y0
            else: 
                x1 = x0
                y1 += get_left_pos()
                
            line = (x0, y0, x1, y1)

            print ("drawLine:" + str(line))

            x0, y0 = x1, y1
            
            # Turn left on the spot
            particles = rotate(particles, math.pi / 2)
            print("drawParticles:" + str(particles))

            time.sleep(2)  # Depends on speed. May need to change after testing
        

    except KeyboardInterrupt:
        BP.reset_all()

    BP.reset_all()