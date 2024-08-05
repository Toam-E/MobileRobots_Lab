import numpy as np
from trajectory import Trajectory
from cspace import CSpace
import car_consts
import matplotlib.pyplot as plt
from kino_rrt import KINORRT
from car_simulator import State, States, Simulator
from pure_pursuit import PurePursuit_Controller
from plot_utils import Plotter, inflate

#  hyper-parameters
k = 0.1  # look forward gain
Lfc = 1.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
target_speed = 1.0  # [m/s]
T = 100.0  # max simulation time
WB = car_consts.wheelbase 
MAX_STEER = car_consts.max_steering_angle_rad  # maximum steering angle [rad]
MAX_DSTEER = car_consts.max_dt_steering_angle  # maximum steering speed [rad/s]
MAX_SPEED = car_consts.max_linear_velocity  # maximum speed [m/s]
MIN_SPEED = car_consts.min_linear_velocity  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

# run params
RUN_KRRT = False
RUN_PP = True

def main():

    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    resolution = 0.05000000074505806
    inflated_map = inflate(map_original, 0.2/resolution)
    converter = CSpace(resolution, origin_x=-4.73, origin_y=-5.66, map_shape=map_original.shape)
    start = [0.0,0.0]
    start_pixel = converter.meter2pixel(start)
    goal = [6.22, -4.22]
    goal_pixel = converter.meter2pixel(goal)
    planned_ktree = None
    if RUN_KRRT:
        cost = None
        while cost is None:
            kinorrt_planner = KINORRT(env_map=inflated_map, max_step_size=20, max_itr=10000, p_bias=0.05,converter=converter )
            path, path_idx, cost = kinorrt_planner.find_path(start_pixel, goal_pixel)
            print(f'cost: {cost}')
            if cost != None:
                path_meter = np.array(converter.pathindex2pathmeter(path))
                np.save(f'krrt_path_pixels.npy', path)
                np.save(f'krrt_path_meters.npy', path_meter)
    else:
        path = np.load('krrt_path_pixels.npy')
        path_meter = np.load('krrt_path_meters.npy')

    if RUN_PP:
        trajectory = Trajectory(dl=0.1, path=path_meter, TARGET_SPEED=target_speed)
        state = State(x=trajectory.cx[0], y=trajectory.cy[0], yaw=trajectory.cyaw[0], v=0.0)
        lastIndex = len(trajectory.cx) - 1
        clock = 0.0
        states = States()
        states.append(clock, state)
        pp = PurePursuit_Controller(trajectory.cx, trajectory.cy, k, Lfc, Kp, WB, MAX_ACCEL, MAX_SPEED, MIN_SPEED, MAX_STEER, MAX_DSTEER)
        target_ind, _, nearest_index = pp.search_target_index(state)
        simulator = Simulator(inflated_map, trajectory, dt)
        closest_path_coords = []
        closest_path_coords.append([trajectory.cx[0], trajectory.cy[0]])
        while T >= clock and lastIndex > target_ind:
            state.v = pp.proportional_control_acceleration(target_speed, state.v, dt)
            delta, target_ind, closest_index = pp.pure_pursuit_steer_control(state, trajectory, dt)
            state.predelta = delta
            state = simulator.update_state(state, delta)  # Control vehicle
            clock += dt
            states.append(clock, state, delta)
            closest_path_coords.append([trajectory.cx[closest_index], trajectory.cy[closest_index]])
        #simulator.show_simulation(states, closest_path_coords)
        states_pixels = states.get_states_in_meters(converter)
        traj_pixels = trajectory.get_trajectory_in_meters(converter)
        closest_path_coords_pixels = converter.pathmeter2pathindex(closest_path_coords)
        simulator.create_animation(states_pixels, traj_pixels, start_pixel, goal_pixel, closest_path_coords_pixels)

if __name__ == '__main__':
    main()
