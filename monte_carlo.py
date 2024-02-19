from __future__ import print_function
from __future__ import division
import time
import brickpi3
from probabilistic_motion import forward, rotate, MOVING_FACTOR
import math
from likelihood import calculate_likelihood

BP = brickpi3.BrickPi3()




motor_left = BP.PORT_D
motor_right = BP.PORT_A

sonar = BP.PORT_4

BP.set_sensor_type(sonar, BP.SENSOR_TYPE.NXT_ULTRASONIC)


BP.set_motor_position_kp(motor_left, 30)
BP.set_motor_position_kd(motor_left, 150)

BP.set_motor_position_kp(motor_right, 30)
BP.set_motor_position_kd(motor_right, 150)

BP.set_motor_limits(motor_left, 50, 200)
BP.set_motor_limits(motor_right, 50, 200)

NUMBER_OF_PARTICLES = 100
OFFSET_Y = 680

MOVING_FACTOR = 17
STEP_SIZE = 20 * MOVING_FACTOR

particles = [(0, OFFSET_Y, 0, 1 / NUMBER_OF_PARTICLES) for _ in range(NUMBER_OF_PARTICLES)]

def get_particle_x():
    return sum([p[0] for p in particles]) / NUMBER_OF_PARTICLES

def get_particle_y():
    return sum([p[1] for p in particles]) / NUMBER_OF_PARTICLES

def get_particle_theta():
    return sum([p[2] for p in particles]) / NUMBER_OF_PARTICLES

def navigate_to_waypoint(Wx, Wy, particles):
    x = get_particle_x()
    y = get_particle_y()
    theta = -get_particle_theta()
    
    dx = (Wx * MOVING_FACTOR) - x
    dy = (-40 - Wy) * MOVING_FACTOR + y
    
    angle = math.atan2(dy, dx)
    d = math.sqrt(dx ** 2 + dy ** 2)

    beta = angle - theta
    if beta <= -math.pi:
        beta += 2 * math.pi
    elif beta > math.pi:
        beta -= 2 * math.pi

    particles = rotate(particles, beta)
    update()
    time.sleep(3)

    BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_right))
    BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right))
    
    # Move forward in steps of 20cm
    while d > STEP_SIZE:
        particles = forward(particles, STEP_SIZE)
        update()
        d -= STEP_SIZE

    particles = forward(particles, d)
    update()
    return particles

def update():
    # Measure z from sonar
    z = 0
    
    # Update weights based on likelihood    
    for p in particles:
        p[3] *= calculate_likelihood(p[0], p[1], p[2], z)
    
    # Normalise weights
    weight_sum = sum([p[3] for p in particles])
    for p in particles:
        p[3] /= weight_sum


    
    

while True:
    x = float(input("Enter x coordinate: "))
    y = float(input("Enter y coordinate: "))

    BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
    BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right))
    particles = navigate_to_waypoint(x, -y, particles)
    time.sleep(1)