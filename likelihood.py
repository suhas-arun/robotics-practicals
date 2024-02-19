from __future__ import print_function
from __future__ import division
import time
import brickpi3
import random
import math
import monte_carlo
import particleDataStructures

wallMap = particleDataStructures.mymap

def get_closest_wall(x,y,theta):
    smallest_m = float('inf')
    closest_wall = wallMap.walls[0]
    for wall in wallMap.walls:
        ax = wall[0]
        ay = wall[1]
        bx = wall[2]
        by = wall[3]
        numerator = (by-ay)*(ax-x) - (bx-ax)*(ay-y)
        denominator = (by-ay)*math.cos(theta) - (bx-ax)*math.sin(theta)
        m = numerator/denominator
        x_intercept = x + m * math.cos(theta)
        y_intercept = y + m * math.sin(theta)
        if (min(ax,bx) <= x_intercept <= max(ax,bx) and min(ay,by) <= y_intercept <= max(ay, by)):
            if m < smallest_m and m >= 0:
                smallest_m = m 
                closest_wall = wall
    return (smallest_m, closest_wall)
          
def calculate_likelihood(x, y, theta, z):
    m, closest_wall = get_closest_wall(x,y,theta)
    std = 2.5 * monte_carlo.MOVING_FACTOR
    k = 0.01
    likelihood = math.exp((-(z-m) ** 2)/(2 *std**2)) + k
    return likelihood