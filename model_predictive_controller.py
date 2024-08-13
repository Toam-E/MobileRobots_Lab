import math
from trajectory import Trajectory
from consts import *
from kino_rrt import KINORRT
from car_simulator import SimState
from cspace import CSpace
from geo_utils import calculate_bounding_box, clip_bounding_box


class ModelPredictiveController(object):
    def __init__(self, obs_map, trajectory : Trajectory, converter: CSpace,\
        krrt_depth=MPC_KRRT_DEPTH, max_search_iterations_per_krrt=MPC_MAX_SEARCH_ITERATIONS_PER_KRRT,\
        lf=MPC_LOCAL_GOAL_LOOK_AHEAD):

        self.obs_map = obs_map
        self.trajectory = trajectory
        self.converter = converter
        self.krrt_depth = krrt_depth
        self.max_search_iterations_per_krrt = max_search_iterations_per_krrt
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

        # look for the L point
        while (ind+1) < len(cx):
            dist = self.calc_distance(state.rear_x, state.rear_y, cx[ind], cy[ind])
            if dist >= self.lf:
                break
            ind += 1
        
        # make sure the local goal point is not on an obstacle, otherwise move even further
        local_goal_was_in_obs = False
        while (ind+1) < len(cx):
            if not self.is_config_in_collision([cx[ind], cy[ind]]):
                break
            local_goal_was_in_obs = True
            ind += 1

        # we move some extra points from the obstacle
        if local_goal_was_in_obs:
            for _ in range(MPC_NUM_OF_INDICES_AFTER_LOCAL_GOAL_OUT_OF_COLLISON):
                if (ind+1) < len(cx):
                    ind += 1

        return ind, self.lf, self.old_nearest_point_index

    def calc_distance(self, rear_x, rear_y, point_x, point_y):
        return math.sqrt((rear_x - point_x)**2 + (rear_y - point_y)**2)

    def calc_search_area(self, local_start, local_goal, curren_state_margin=10):
        min_x, max_x, min_y, max_y = calculate_bounding_box(local_start, local_goal, curren_state_margin)
        return clip_bounding_box(min_x, max_x, min_y, max_y, self.obs_map.shape)
    
    def is_config_in_collision(self, conf):
        pixel = self.converter.meter2pixel(conf)
        return self.obs_map[pixel[1], pixel[0]] != 0

    def convert_local_path_to_global_map(self, x_min, y_min, path):
        new_path = [ [elem[0] + x_min, elem[1] + y_min] for elem in path]
        return new_path

    def predict(self, state : SimState, dt):
        list_of_state_pixel = self.converter.meter2pixel([state.x, state.y, state.yaw])
        local_start_pixel = [list_of_state_pixel[0], list_of_state_pixel[1]]
        target_ind, _, _ = self.search_local_goal_index(state)
        local_goal = [self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]]
        state.mpc_local_goal = local_goal
        state.mpc_local_goal_pixel = self.converter.meter2pixel(local_goal)
        state.mpc_bbox = self.calc_search_area(local_start_pixel, state.mpc_local_goal_pixel)
        x_min, x_max, y_min, y_max= state.mpc_bbox
        cropped_area = self.obs_map[y_min:y_max, x_min:x_max]
        new_start = [local_start_pixel[0] - x_min, local_start_pixel[1] - y_min]
        new_goal = [state.mpc_local_goal_pixel[0] - x_min, state.mpc_local_goal_pixel[1] - y_min]

        if self.is_config_in_collision([state.x, state.y]):
            print("local start state is already in collision, no KRRT done")
            return

        min_cost_path = None
        krrt = KINORRT(cropped_area, max_itr=self.max_search_iterations_per_krrt,\
            p_bias=0.1, converter=self.converter, goal_radius=7)
        krrt_paths = []
        krrt_costs = []
        for _ in range(MPC_NUM_OF_PATHS_TO_CMP):
            krrt.reset()
            path, _, cost = krrt.find_path(new_start, new_goal)
            if path is not None:
                krrt_paths.append(path)
                krrt_costs.append(cost)

        if len(krrt_paths) > 0:
            min_cost_index = krrt_costs.index(min(krrt_costs))
            min_cost_path = krrt_paths[min_cost_index]
            state.krrt_path = self.convert_local_path_to_global_map(x_min, y_min, min_cost_path)
            print("local plan found")
        else:
            print("No local plan found")
        return min_cost_path


