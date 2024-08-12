import math
from trajectory import Trajectory
from consts import *
from kino_rrt import KINORRT
from car_simulator import SimState
from cspace import CSpace
from geo_utils import calculate_bounding_box, clip_bounding_box


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
        target_ind, _, _ = self.search_local_goal_index(state)
        local_goal = [self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]]
        state.mpc_local_goal = local_goal
        state.mpc_local_goal_pixel = self.converter.meter2pixel(local_goal)
        state.mpc_bbox = self.calc_search_area(state)

