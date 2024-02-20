from __future__ import print_function
from __future__ import division
import time
import brickpi3
import math
from likelihood import calculate_likelihood
import random

BP = brickpi3.BrickPi3()

motor_left = BP.PORT_D
motor_right = BP.PORT_A

# sonar = BP.PORT_4

BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

BP.set_motor_position_kp(motor_left, 30)
BP.set_motor_position_kd(motor_left, 150)

BP.set_motor_position_kp(motor_right, 30)
BP.set_motor_position_kd(motor_right, 150)

BP.set_motor_limits(motor_left, 50, 200)
BP.set_motor_limits(motor_right, 50, 200)

TURNING_FACTOR = 117 # How much to turn to rotate 1 radian
MOVING_FACTOR = 17 # How much to move forward 1 cm
STEP_SIZE = 20 * MOVING_FACTOR

MAP_SIZE = 210 # Size in cm of map
CANVAS_SIZE = 768 # Size in pixels of canvas display
MARGIN = 0.05 * CANVAS_SIZE

NUMBER_OF_PARTICLES = 100
OFFSET_Y = MAP_SIZE * MOVING_FACTOR

SCALE = CANVAS_SIZE / OFFSET_Y

# Initial position
INIT_X = 20
INIT_Y = 20

particles = [(INIT_X, OFFSET_Y - INIT_Y, 0, 1 / NUMBER_OF_PARTICLES) for _ in range(NUMBER_OF_PARTICLES)]

def draw_particles(data):
    particles = [
        (p[0] * SCALE + MARGIN, p[1] * SCALE - MARGIN, p[2], p[3]) for p in data]
    print("drawParticles:" + str(particles))

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

        particles[j] = (particle_x_new, particle_y_new, theta_new, particles[j][3])

    return particles

def rotate(particles, angle):
    turning_distance = TURNING_FACTOR * angle
    BP.set_motor_position(motor_left, BP.get_motor_encoder(motor_left) - turning_distance)
    BP.set_motor_position(motor_right, BP.get_motor_encoder(motor_right) + turning_distance)
    return update_particles_after_rotation(particles, angle)


def update_particles_after_rotation(particles, angle):
    for j in range(NUMBER_OF_PARTICLES):
        g = random.gauss(0, 0.01) # rotational spread

        particle_x_new = particles[j][0]
        particle_y_new = particles[j][1]
        theta_new = particles[j][2] - angle + g

        particles[j] = (particle_x_new, particle_y_new, theta_new, particles[j][3])

    return particles

def get_particle_x(particles):
    return sum([p[0] * p[3] for p in particles])

def get_particle_y(particles):
    return sum([p[1] * p[3] for p in particles])

def get_particle_theta(particles):
    theta = sum([p[2] * p[3] for p in particles])
    return theta % (2 * math.pi)

def navigate_to_waypoint(Wx, Wy, particles):
    x = get_particle_x(particles)
    y = get_particle_y(particles)
    theta = -get_particle_theta(particles)

    dx = (Wx * MOVING_FACTOR) - x
    dy = (-MAP_SIZE - Wy) * MOVING_FACTOR + y
    
    angle = math.atan2(dy, dx)
    d = math.sqrt(dx ** 2 + dy ** 2)

    beta = angle - theta
    if beta <= -math.pi:
        beta += 2 * math.pi
    elif beta > math.pi:
        beta -= 2 * math.pi

    particles = rotate(particles, beta)
    particles = update(particles)
    time.sleep(1)

    BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_right))
    BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right))
    
    # Move forward in steps of 20cm
    while d > STEP_SIZE:
        particles = forward(particles, STEP_SIZE)
        particles = update(particles)
        d -= STEP_SIZE
        time.sleep(1)

    particles = forward(particles, d)
    particles = update(particles)
    return particles

def update(particles):
    # Measure z from sonar
    z = read_sensor()
    #z = 20
    print(f"Sensor reading: {z}")

    # Update weights based on likelihood
    for i, p in enumerate(particles):
        new_weight = p[3] * calculate_likelihood(p[0], p[1], p[2], z)
        particles[i] = (p[0], p[1], p[2], new_weight)
    
    # Normalise weights
    weight_sum = sum([p[3] for p in particles])
    for i, p in enumerate(particles):
        normalised_weight = p[3] / weight_sum
        particles[i] = (p[0], p[1], p[2], normalised_weight)

    draw_particles(particles)
    particles = resample(particles)
    time.sleep(1)
    draw_particles(particles)
    time.sleep(1)
    return particles
    
def resample(particles):
    print("Resampling")
    cum_weights = [0 for _ in range(NUMBER_OF_PARTICLES)]
    for i in range(NUMBER_OF_PARTICLES):
        cum_weights[i] = particles[i][3] + (cum_weights[i - 1] if i > 0 else 0)
    
    new_particles = []
    for _ in range(NUMBER_OF_PARTICLES):
        rand = random.random()
        index = 0
        for weight in cum_weights:
            if rand <= weight:
                p = particles[index]
                new_particles.append((p[0], p[1], p[2], 1 / NUMBER_OF_PARTICLES))
                break
            else:
                index += 1

    return new_particles
    
def read_sensor():
    # time.sleep(0.5)
    fails = 0 
    while True:
        try:
            value = BP.get_sensor(BP.PORT_1)
            return value
        except brickpi3.SensorError as error:
            fails += 1
            # print(f"Failed: {fails}")
            time.sleep(0.02)
        

if __name__ == "__main__":
    waypoints = [(84, 30), (180, 30), (180, 54), (138, 54), (138, 168), (114, 168), (114, 84), (84, 84), (84, 30)]
    for x, y in waypoints:
        print(f"Navigating to ({x}, {y})")
        BP.offset_motor_encoder(motor_left, BP.get_motor_encoder(motor_left))
        BP.offset_motor_encoder(motor_right, BP.get_motor_encoder(motor_right))
        draw_particles(particles)
        particles = navigate_to_waypoint(x, -y, particles)
        time.sleep(2)