import numpy as np
import copy
from trajectory import Trajectory
from cspace import CSpace
import car_consts
import matplotlib.pyplot as plt
from kino_rrt import KINORRT
from car_simulator import SimState, SimStatesContainer, Simulator
#from pure_pursuit import PurePursuit_Controller
from local_planner import LocalPlanner
from plot_utils import Plotter, plot_map
from utils import inflate, add_new_obstacles
from combined_controller import CombinedController, CONTROLLER_STATES_KRRT, CONTROLLER_STATES_PP

#  hyper-parameters
LF_K = 0.1  # look forward gain
LFC = 1.0  # [m] look-ahead distance
V_KP = 1.0  # speed proportional gain
DELTA_T = 0.1  # [s] time tick
TARGETED_SPEED = 1.0  # [m/s]
T = 100.0  # max simulation time
WB = car_consts.wheelbase 
MAX_STEER = car_consts.max_steering_angle_rad  # maximum steering angle [rad]
MAX_DSTEER = car_consts.max_dt_steering_angle  # maximum steering speed [rad/s]
MAX_SPEED = car_consts.max_linear_velocity  # maximum speed [m/s]
MIN_SPEED = car_consts.min_linear_velocity  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

# run params
RUN_KRRT        = False
RUN_ADD_OBS     = True
RUN_PP          = True
RUN_ANIMATION   = True



def main():

    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    resolution = 0.05000000074505806
    inflation = 0.2/resolution
    inflated_map = inflate(map_original, inflation)
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
        # path = np.load('path_maze_kinorrtpixels.npy')
        # path_meter = np.load('path_maze_kinorrtmeters.npy')
        path = np.load('krrt_path_pixels.npy')
        path_meter = np.load('krrt_path_meters.npy')

    if RUN_ADD_OBS:
        # add on path new obstacles
        path_fractions = [0.2, 0.4, 0.6, 0.8]
        inflations = [4, 4, 4, 4]
        # inflations = [4, 5, 5, 5]
        new_obs_map = add_new_obstacles(inflated_map, path, path_fractions, inflations)
        #plt.imshow(new_obs_map, origin="lower")
        #plt.show()
    else:
        new_obs_map = inflated_map


    if RUN_PP:
        try:
            controller = CombinedController(path_meter, new_obs_map, converter = converter)
            controller.find_path(goal_pixel)
        # trajectory = Trajectory(dl=0.1, path=path_meter, TARGET_SPEED=TARGETED_SPEED)
        # state = SimState(x=trajectory.cx[0], y=trajectory.cy[0], yaw=trajectory.cyaw[0], v=0.0)
        # lastIndex = len(trajectory.cx) - 1
        # clock = 0.0
        # states = SimStatesContainer()
        # state.t = clock
        # states.append(state)
        # lp = LocalPlanner(converter, new_obs_map, trajectory.cx, trajectory.cy,\
        #                   LF_K, LFC, V_KP, WB, MAX_ACCEL, MAX_SPEED,\
        #                   MIN_SPEED, MAX_STEER, MAX_DSTEER)
        # krrt_planner = KINORRT(env_map=new_obs_map, max_step_size=KINORRT_MAX_STEP_SIZE, max_itr=KINORRT_ITER, p_bias=0.05,converter=converter )
        # target_ind, _, nearest_index = lp.search_target_index(state)
        # simulator = Simulator(new_obs_map, trajectory, DELTA_T)
        # closest_path_coords = []
        # closest_path_coords.append([trajectory.cx[0], trajectory.cy[0]])
        # while T >= clock and lastIndex > target_ind:
        #     state = copy.copy(state)
        #     if lp.local_obs_detected(state, cone_radius=10, cone_fov=np.pi/3):
        #         pass
        #         optional_routes = []

        #         # run KRRT locally and find new optional routes
        #         krrt_start = (state.x, state.y)
        #         for i in range(KINORRT_TRIES):
        #             curr_route, _, curr_cost = krrt_planner._find_path(krrt_start, goal_pixel)
        #             if curr_route is not None:
        #                 optional_routes.append((curr_route, curr_cost))
                    
                
                # choose route
                # grade_routes(optional_routes, trajectory, new_obs_map)

                # pure pursuit according to new trajectory
                # 

                # pass
            # maybe some of it shouldn't be in else
            # pure_pursuit_update_state(state, trajectory, lp, simulator, states, closest_path_coords)
            # state.v = lp.proportional_control_acceleration(TARGETED_SPEED, state.v, DELTA_T)
            # delta, target_ind, closest_index = lp.pure_pursuit_steer_control(state, trajectory, DELTA_T)
            # state.predelta = delta
            # state = simulator.update_state(state, delta)  # Control vehicle
            # clock += DELTA_T
            # state.t = clock
            # state.a = delta
            # states.append(state)
            # closest_path_coords.append([trajectory.cx[closest_index], trajectory.cy[closest_index]])
        except :
            pass
        if RUN_ANIMATION:
            controller.set_controller_state(CONTROLLER_STATES_PP)
            states_pixels = controller.states.get_states_in_pixels(controller.converter)
            traj_pixels = controller.main_trajectory.get_trajectory_in_pixels(controller.converter)
            closest_path_coords_pixels = controller.converter.pathmeter2pathindex(controller.closest_path_coords)
            states_pixels.calc_states_cones()

            controller.simulator.show_simulation(controller.states, controller.krrt_traj_list_meters, controller.krrt.samples, controller.closest_path_coords)
            controller.simulator.create_animation(states_pixels, traj_pixels, controller.krrt_traj_list_pixels, controller.krrt.samples, start_pixel, goal_pixel, closest_path_coords_pixels)

if __name__ == '__main__':
    main()

