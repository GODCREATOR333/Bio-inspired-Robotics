import random
import math

def random_point_outside_radius(min_radius, max_radius):
        while True:
            x = random.uniform(-max_radius, max_radius)
            y = random.uniform(-max_radius, max_radius)

            if math.sqrt(x*x + y*y) >= min_radius:
                return x, y
            