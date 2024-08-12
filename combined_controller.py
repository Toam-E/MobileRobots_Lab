import numpy as np
import copy
from consts import *
from trajectory import Trajectory
from cspace import CSpace
from kino_rrt import KINORRT
from local_planner import LocalPlanner
from car_simulator import SimState, SimStatesContainer, Simulator

CONTROLLER_STATES_PP    = 1
CONTROLLER_STATES_KRRT  = 2

class CombinedController(object):

    def __init__(self, path_meter, env_map, converter = None, \
                 krrt_tries = KINORRT_TRIES, krrt_iters = KINORRT_ITER, krrt_max_step = KINORRT_MAX_STEP_SIZE):
        self.map = env_map
        if converter is None:
            converter = CSpace(resolution, origin_x=-4.73, origin_y=-5.66, map_shape=env_map.shape)
        self.converter = converter
        
        self.krrt_tries = krrt_tries
        self.krrt_iters = krrt_iters
        self.krrt_max_step = krrt_max_step

        self.orig_trajectory = Trajectory(dl=0.1, path=path_meter, TARGET_SPEED=TARGETED_SPEED)
        self.temp_trajectory = Trajectory(dl=0.1, path=path_meter, TARGET_SPEED=TARGETED_SPEED) # trajectory to switch to when needed
        self.main_trajectory = copy.copy(self.orig_trajectory)

        self.simulator = Simulator(self.map, self.orig_trajectory, DELTA_T)

        self.clock  = 0.0
        self.states = SimStatesContainer()
        self.state  = SimState(x=self.orig_trajectory.cx[0], y=self.orig_trajectory.cy[0], yaw=self.orig_trajectory.cyaw[0], v=0.0)
        self.state.t = self.clock
        self.states.append(copy.copy(self.state))

        self.krrt           = KINORRT(env_map=self.map, max_step_size=self.krrt_max_step, max_itr=self.krrt_iters,\
                                       p_bias=0.05,converter=self.converter)
        
        self.overtake_lp    = LocalPlanner(self.converter, self.map, self.temp_trajectory.cx, self.temp_trajectory.cy,\
                                           LF_K, LFC, V_KP, WB, MAX_ACCEL, MAX_SPEED,\
                                            MIN_SPEED, MAX_STEER, MAX_DSTEER)
        self.orig_lp        = LocalPlanner(self.converter, self.map, self.orig_trajectory.cx, self.orig_trajectory.cy,\
                                           LF_K, LFC, V_KP, WB, MAX_ACCEL, MAX_SPEED,\
                                            MIN_SPEED, MAX_STEER, MAX_DSTEER)
        self.lp             = copy.copy(self.orig_lp)
        
        self.controller_state = CONTROLLER_STATES_PP
        
        self.target_ind = 0 # self.lp.search_target_index(self.state)[0]
        self.main_target_ind = self.target_ind
        self.goal_ind = self.target_ind


        self.closest_path_coords = []
        self.closest_path_coords.append([self.orig_trajectory.cx[0], self.orig_trajectory.cy[0]])

        self.krrt_traj_list_meters = []
        self.krrt_traj_list_pixels = []


    @property
    def lastIndex(self):
        return len(self.main_trajectory.cx) - 1

    def pure_pursuit_update_state(self):
        self.state.v = self.lp.proportional_control_acceleration(TARGETED_SPEED, self.state.v, DELTA_T)
        delta, self.target_ind, closest_index = self.lp.pure_pursuit_steer_control(self.state, self.main_trajectory, DELTA_T)
        self.state.predelta = delta
        self.state = self.simulator.update_state(self.state, delta)  # Control vehicle
        self.clock += DELTA_T
        self.state.t = self.clock
        self.state.a = delta
        self.states.append(copy.copy(self.state))
        self.closest_path_coords.append([self.main_trajectory.cx[closest_index], self.main_trajectory.cy[closest_index]])
        return

    def get_best_trajectory(self, optional_trajectories):
        # for now, return the first one 
        # by distance from original trajectory
        # later - by distance from obstacles in the way - not too close to obstacles.
        print(f"len(optional_trajectories): {len(optional_trajectories)}")
        print(optional_trajectories)
        print(f"{optional_trajectories[0] == optional_trajectories[1]} {optional_trajectories[1] == optional_trajectories[2]} {optional_trajectories[2] == optional_trajectories[3]}")
        lowest_cost_idx = 0
        min_cost = 0
        for idx, (path, cost) in enumerate(optional_trajectories):
            if len(path) < 2:
                cost = cost + 20000
            if min_cost > cost:
                lowest_cost_idx = idx
                min_cost = cost

        return optional_trajectories[lowest_cost_idx]
    
    def get_krrt_path_meter(self, goal_pixel):
        optional_trajectories = []

        # run KRRT locally and find new optional trajectories
        krrt_start_pixel = self.converter.meter2pixel((self.state.x, self.state.y, self.state.yaw))
        for i in range(KINORRT_TRIES):
            self.krrt.reset()
            curr_trajectory, _, curr_cost = self.krrt._find_path(krrt_start_pixel, goal_pixel)
            if curr_trajectory is not None:
                optional_trajectories.append((curr_trajectory, curr_cost))

        # choose best trajectory
        best_krrt_trajectory, cost = self.get_best_trajectory(optional_trajectories)
        # TODO: THIS probably is THE problem!!
        while len (best_krrt_trajectory) < 2: #TODO: this is probably wrong
            print("inserting point to best_krrt_trajectory")
            best_krrt_trajectory.insert(0, self.converter.meter2pixel(self.closest_path_coords[-1]))
        return self.converter.pathindex2pathmeter(best_krrt_trajectory)
        # return best_krrt_trajectory

    def find_path(self, goal_pixel):
        # the main part of the controller
        print(f"in find path goal_pixel {goal_pixel}")
        while T >= self.clock and self.lastIndex > self.target_ind:
            # state = copy.copy(state) # TODO: what is this?
            if self.lp.local_obs_detected(self.state, cone_radius=8, cone_fov=np.pi/4):
                # prepare env for state switch                    
                if self.target_ind + NEXT_IDX < self.lastIndex:
                    self.goal_idx = (self.target_ind + NEXT_IDX)
                    print(f"self.goal_idx {self.goal_idx} self.target_ind {self.target_ind}")
                else:
                    self.goal_idx = self.lastIndex
                # goal_pixel_idx = (self.target_ind + NEXT_IDX) if (self.target_ind + NEXT_IDX < self.lastIndex) else self.lastIndex
                goal_pixel = self.converter.meter2pixel((self.orig_trajectory.cx[self.goal_idx], self.orig_trajectory.cy[self.goal_idx]))
                print(f"before get_krrt_path_meter goal_pixel {goal_pixel}")

                # run KRRT locally and find best new trajectory
                best_krrt_trajectory = self.get_krrt_path_meter(goal_pixel) # TODO: change to local goal_pixel!
                print(best_krrt_trajectory)
                self.temp_trajectory = Trajectory(dl=0.1, path=best_krrt_trajectory, TARGET_SPEED=TARGETED_SPEED)
                self.krrt_traj_list_meters.append(copy.copy(self.temp_trajectory))
                self.krrt_traj_list_pixels.append(copy.copy(self.temp_trajectory.get_trajectory_in_pixels(self.converter)))
                # print(self.converter.pathindex2pathmeter(self.temp_trajectory.path))

                # switch state in order to pure pursuit according to new trajectory
                self.set_controller_state(CONTROLLER_STATES_KRRT)
                while (not self.finished_trajectory()):
                    self.pure_pursuit_update_state()

                self.set_controller_state(CONTROLLER_STATES_PP)

            # elif (not self.finished_local_trajectory()): 
            #     pass
            
            # maybe some of it shouldn't be in else
            else: # update according to original trajectory
                self.set_controller_state(CONTROLLER_STATES_PP)
                self.pure_pursuit_update_state()
        if T < self.clock:
            raise Exception("ran out of time!")
    
    def set_controller_state(self, c_state):
        if self.controller_state != c_state:
            print(f"switching to state {c_state}")
            self.controller_state = c_state
            if CONTROLLER_STATES_PP == self.controller_state: # switching to PP
                # self.target_ind         = self.main_target_ind
                self.main_trajectory    = self.orig_trajectory
                self.lp                 = self.orig_lp
                self.target_ind         = min(max(self.lp.search_target_index(self.state)[0] \
                                                  + round(self.lp.Lfc)\
                                                  , self.main_target_ind), self.lastIndex)
                # self.target_ind         = min(max(round(np.mean(self.goal_ind, self.main_target_ind)), self.main_target_ind), self.lastIndex)
            else: # self.controller_state == CONTROLLER_STATES_KRRT
                self.main_target_ind    = self.target_ind
                self.target_ind         = 0
                self.main_trajectory    = self.temp_trajectory
                # self.lp                 = self.overtake_lp
                self.lp                 = LocalPlanner(self.converter, self.map, self.temp_trajectory.cx, self.temp_trajectory.cy,\
                                           LF_K, LFC, V_KP, WB, MAX_ACCEL, MAX_SPEED,\
                                            MIN_SPEED, MAX_STEER, MAX_DSTEER)
                self.krrt               = KINORRT(env_map=self.map, max_step_size=self.krrt_max_step, max_itr=self.krrt_iters,\
                                       p_bias=0.05,converter=self.converter)

    
    def finished_trajectory(self):
        return self.lastIndex <= self.target_ind