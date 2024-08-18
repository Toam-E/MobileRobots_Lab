import numpy as np
from cspace import CSpace

class Odom(object):
    def __init__(self, converter:CSpace):
        self.wheelbase = 0.35
        self.max_steering_angle = np.deg2rad(35)
        self.min_velocity, self.max_velocity = 0.5, 1
        self.min_time, self.max_time = 1, 2
        self.converter = converter
    
    def sample_control_command(self):
        delta_time = np.random.uniform(self.min_time, self.max_time)
        steering = np.random.uniform(-self.max_steering_angle, self.max_steering_angle)
        velocity = np.random.uniform(self.min_velocity, self.max_velocity)
        return delta_time, steering, velocity

    def propagate(self,  steering, velocity ,delta_time, initial_x):
        initial_x = self.converter.pixel2meter(initial_x)
        x = initial_x[0]
        y = initial_x[1]
        theta= initial_x[2]
        theta_dot = velocity * np.tan(steering) / self.wheelbase
        dt = 0.03
        edge = [[x,y,theta]]
        cost = 0
        for _ in range(int(delta_time/dt)):
            theta += theta_dot * dt
            x_dot = velocity * np.cos(theta)
            y_dot = velocity * np.sin(theta)
            x += x_dot * dt
            y += y_dot * dt
            cost += ((edge[-1][0] - x)**2 + (edge[-1][1] - y)**2)**0.5
            edge.append([x,y,theta])
        edge = self.converter.pathmeter2pathindex(edge)
        new_state = edge[-1]
        return new_state, edge, cost

