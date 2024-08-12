import copy
import math
from trajectory import Trajectory
from consts import *
from kino_rrt import KINORRT
from car_simulator import SimState, SimStatesContainer, Simulator
from pure_pursuit import PurePursuitController
from cspace import CSpace
from geo_utils import calculate_bounding_box, clip_bounding_box

MPC_KRRT_DEPTH = 3
MPC_MAX_SEARCH_ITERATIONS = 100
MPC_NUM_OF_PATHS_TO_CMP = 3
MPC_ITEREATIONS = 3
MPC_LOCAL_GOAL_LOOK_AHEAD = 2.0

class ModelPredictiveController(object):
    def __init__(self, obs_map, trajectory : Trajectory, converter: CSpace,\
        krrt_depth=MPC_KRRT_DEPTH, max_search_iterations=MPC_MAX_SEARCH_ITERATIONS,\
        lf=MPC_LOCAL_GOAL_LOOK_AHEAD):

        self.obs_map = obs_map
        self.trajectory = trajectory
        self.converter = converter
        self.krrt_depth = krrt_depth
        self.max_search_iterations = max_search_iterations
        self.lf = lf
        self.old_nearest_point_index = None

    def search_local_goal_index(self, state: SimState):

        cx = self.trajectory.cx
        cy = self.trajectory.cy
        min_distance = math.inf
        min_distance_idx = None
        for idx, _ in enumerate(cx):
            current_dist = \
                self.calc_distance(state.rear_x, state.rear_y, cx[idx], cy[idx])
            if current_dist < min_distance:
                min_distance = current_dist
                min_distance_idx = idx

        assert(min_distance_idx != None)
        self.old_nearest_point_index = min_distance_idx

        ind = self.old_nearest_point_index
        while (ind+1) < len(cx):
            dist = self.calc_distance(state.rear_x, state.rear_y, cx[ind], cy[ind])
            if dist >= self.lf:
                break
            ind += 1
        return ind, self.lf, self.old_nearest_point_index

    def calc_distance(self, rear_x, rear_y, point_x, point_y):
        return math.sqrt((rear_x - point_x)**2 + (rear_y - point_y)**2)

    def calc_search_area(self, state:SimState, curren_state_margin=10):
        list_of_state_pixel = self.converter.meter2pixel([state.x, state.y, state.yaw])
        state_position_pixel = [list_of_state_pixel[0], list_of_state_pixel[1]]
        min_x, max_x, min_y, max_y = calculate_bounding_box(state_position_pixel,\
            state.mpc_local_goal_pixel, curren_state_margin)
        return clip_bounding_box(min_x, max_x, min_y, max_y, self.obs_map.shape)

    def predict(self, state:SimState, dt):
        target_ind, _, closest_index = self.search_local_goal_index(state)
        local_goal = [self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]]
        state.mpc_local_goal = local_goal
        state.mpc_local_goal_pixel = self.converter.meter2pixel(local_goal)
        state.mpc_bbox = self.calc_search_area(state)


class CombinedPlanner(object):

    def __init__(self, obs_map, trajectory : Trajectory, converter: CSpace,\
                  sim : Simulator, states : SimStatesContainer):

        self.lp = PurePursuitController(converter, obs_map, trajectory.cx, trajectory.cy,\
                            LF_K, LFC, V_KP, WB, MAX_ACCEL, MAX_SPEED,\
                            MIN_SPEED, MAX_STEER, MAX_DSTEER)
        self.mpc = ModelPredictiveController(obs_map, trajectory, converter)
        self.trajectory = trajectory
        self.sim = sim
        self.states = states
        self.lastIndex = len(trajectory.cx) - 1
        self.clock = 0.0
        self.target_path_coords = []
        self.closest_path_coords = []
        self.mpc_mode = False
        self.mpc_curr_iteration = 0

    def find_path(self):
        cur_state = SimState(x=self.trajectory.cx[0], y=self.trajectory.cy[0], yaw=self.trajectory.cyaw[0], v=0.0)
        cur_state.t = self.clock
        self.states.append(cur_state)
        target_ind, _, _ = self.lp.search_target_index(cur_state)
        self.target_path_coords.append([self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]])
        self.closest_path_coords.append([self.trajectory.cx[0], self.trajectory.cy[0]])

        state_idx = 0
        while T >= self.clock and self.lastIndex > target_ind:
            cur_state = copy.copy(cur_state)
            cur_state.reset_mpc_related()
            state_idx +=1

            if self.lp.local_obs_detected(cur_state, cone_radius=SENSE_CONE_RADIUS, cone_fov=SENSE_CONE_ANGLE):
                self.mpc_mode = True
                self.mpc_curr_iteration = 0

            cur_state.v = self.lp.proportional_control_acceleration(TARGETED_SPEED, cur_state.v, DELTA_T)

            if self.mpc_mode:
                # TODO: once finalized MPC w/ KRRT imp inside, we will replace the PP w/ it.
                # for now, we just want to statistics and plots over what it does
                delta, target_ind, closest_index = self.lp.pure_pursuit_steer_control(cur_state, self.trajectory, DELTA_T)
                if self.mpc_curr_iteration < MPC_ITEREATIONS:
                    self.mpc.predict(cur_state, DELTA_T)
                    self.mpc_curr_iteration += 1
                else:
                    self.mpc_mode = False
                    self.mpc_curr_iteration = 0
                #delta, target_ind, closest_index = self.mpc.predict(cur_state, DELTA_T)
            else:
                delta, target_ind, closest_index = self.lp.pure_pursuit_steer_control(cur_state, self.trajectory, DELTA_T)

            cur_state.predelta = delta
            cur_state = self.sim.update_state(cur_state, delta)  # Control vehicle
            self.clock += DELTA_T
            cur_state.t = self.clock
            cur_state.a = delta
            self.states.append(cur_state)
            self.closest_path_coords.append([self.trajectory.cx[closest_index], self.trajectory.cy[closest_index]])
            self.target_path_coords.append([self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]])

