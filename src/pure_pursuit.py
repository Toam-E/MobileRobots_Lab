import numpy as np
import math
from trajectory import Trajectory
import matplotlib.pyplot as plt
from car_simulator import SimState, SimStatesContainer
from cspace import CSpace


class PurePursuitController(object):
    def __init__(self, converter: CSpace, map, cx, cy,\
                 k=0.1, Lfc=0.5, Kp =1.0, WB =0.335,\
                 MAX_ACCEL=1.0, MAX_SPEED=3, MIN_SPEED=-3,\
                 MAX_STEER=np.deg2rad(27.0), MAX_DSTEER=np.deg2rad(150.0) ):
        self.converter = converter
        self.map = map
        #self.krrt = KINORRT(env_map=map, max_step_size=20, max_itr=100, p_bias=0.1,converter=converter)
        self.k = k  # look forward gain
        self.Lfc = Lfc   # [m] look-ahead distance
        self.Kp = Kp  # speed proportional gain
        self.WB = WB # wheelbase
        self.old_nearest_point_index = None
        self.cx = cx # trajectory x coords
        self.cy = cy # trajectory y coords
        self.MAX_ACCEL = MAX_ACCEL
        self.MAX_SPEED  = MAX_SPEED
        self.MIN_SPEED = MIN_SPEED
        self.MAX_STEER = MAX_STEER
        self.MAX_DSTEER = MAX_DSTEER

    def pure_pursuit_steer_control(self, state: SimState, trajectory: Trajectory,  dt):
        '''
        input: 
        current state,
        trajectory,
        dt
        Return the target index, look ahead distance, and closest index of path coords
        '''
        target_index, Lf, closest_index = self.search_target_index(state)

        # calculate alpha angle based on L line relative to car coordinate position
        # as if it was paraller to world X axis
        alpha = math.atan2(self.cy[target_index] - state.rear_y, self.cx[target_index] - state.rear_x)
        # reduce the current theta to get the real world alpha angle
        alpha -=  state.yaw

        # use the formula that delta = arctan(2 * WB * sin(alpha) / L)
        theta = math.atan2(2.0 * self.WB * math.sin(alpha) , Lf)

        max_change_in_theta = self.MAX_DSTEER * dt
        theta = np.clip(theta, state.predelta - max_change_in_theta, state.predelta + max_change_in_theta)
        theta = np.clip(theta, -self.MAX_STEER, self.MAX_STEER)

        return theta, target_index, closest_index
    
    def proportional_control_acceleration(self, target_speed, current_speed, dt=0.1):
        '''
        returns updated linear speed
        '''
        acc = self.Kp * (target_speed - current_speed)
        acc = np.clip(acc, -self.MAX_ACCEL, self.MAX_ACCEL)
        linear_velocity = current_speed + acc * dt
        linear_velocity = np.clip(linear_velocity, self.MIN_SPEED, self.MAX_SPEED)
        return linear_velocity
        

    def search_target_index(self, state: SimState):
        '''
        input: current state
        output:
        target index, 
        updated look-ahead distance,
        index of closest coords on trajectory 
        '''
        min_distance = math.inf
        min_distance_idx = None
        for idx, _ in enumerate(self.cx):
            current_dist = \
                self.calc_distance(state.rear_x, state.rear_y, self.cx[idx], self.cy[idx])
            if current_dist < min_distance:
                min_distance = current_dist
                min_distance_idx = idx

        assert(min_distance_idx != None)
        self.old_nearest_point_index = min_distance_idx

        Lf = self.Lfc + self.k * state.v

        ind = self.old_nearest_point_index
        while (ind+1) < len(self.cx):
            dist = self.calc_distance(state.rear_x, state.rear_y, self.cx[ind], self.cy[ind])
            if dist >= Lf:
                break
            ind += 1

        return ind, Lf, self.old_nearest_point_index

    def calc_distance(self, rear_x, rear_y, point_x, point_y):
        '''
        calculates the distance between two coords
        '''
        return math.sqrt((rear_x - point_x)**2 + (rear_y - point_y)**2)


def plot_error(closest_path_coords, states:SimStatesContainer, trajectory:Trajectory):
    fig = plt.figure()
    ax = fig.add_subplot()
    total_tracking_error = 0
    for i in range(len(states.x)-1):
        tracking_error = ((closest_path_coords[i][0] -states.rear_x[i])**2 + (closest_path_coords[i][1] - states.rear_y[i])**2)**0.5
        total_tracking_error += tracking_error
        ax.scatter(i, tracking_error, c='b')
    print(f'average tracking error {total_tracking_error/(i+1)}')
    ax.set_xlabel('itr')
    ax.set_ylabel('Tracking Error')
    ax.grid()
    plt.show()
