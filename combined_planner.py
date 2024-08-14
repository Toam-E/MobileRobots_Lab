import copy
from trajectory import Trajectory
from consts import *
from car_simulator import SimState, SimStatesContainer, Simulator
from pure_pursuit import PurePursuitController
from model_predictive_controller import ModelPredictiveController
from cspace import CSpace
from utils import get_normalized_angle
from geo_utils import *

class CombinedPlanner(object):

    def __init__(self, obs_map, trajectory : Trajectory, converter: CSpace,\
                  sim : Simulator, states : SimStatesContainer):

        self.lp = PurePursuitController(converter, obs_map, trajectory.cx, trajectory.cy,\
                            LF_K, LFC, V_KP, WB, MAX_ACCEL, MAX_SPEED,\
                            MIN_SPEED, MAX_STEER, MAX_DSTEER)
        self.mpc = ModelPredictiveController(obs_map, trajectory, converter)
        self.obs_map = obs_map
        self.converter = converter
        self.trajectory = trajectory
        self.sim = sim
        self.states = states
        self.lastIndex = len(trajectory.cx) - 1
        self.clock = 0.0
        self.target_path_coords = []
        self.closest_path_coords = []
        self.mpc_mode = False
        self.mpc_curr_iteration = 0

    def local_obs_detected(self, state: SimState, cone_radius = 15, cone_fov=np.pi/3):
        state_meter_list = [state.x, state.y, state.yaw]
        state_pixel = self.converter.meter2pixel(state_meter_list)
        cone_origin_x = state_pixel[0]
        cone_origin_y = state_pixel[1]
        cone_origin_yaw = state_pixel[2]
        fov_angles = np.linspace(start=cone_fov/2, stop=-cone_fov/2, num=cone_radius)
        tile_yaw = np.tile(cone_origin_yaw, fov_angles.size)
        fov_angles = np.expand_dims(tile_yaw + fov_angles, axis=0)
        car_angles = np.apply_along_axis(get_normalized_angle, 0, fov_angles)
        car_cone_xs = cone_origin_x + cone_radius * np.cos(car_angles)
        car_cone_ys = cone_origin_y + cone_radius * np.sin(car_angles)
        car_cone_xs = np.append(np.insert(car_cone_xs, 0, cone_origin_x), cone_origin_x)
        car_cone_ys = np.append(np.insert(car_cone_ys, 0, cone_origin_y), cone_origin_y)
        # check if cone is in collision
        cone_points = [list(car_cone_point) for car_cone_point in zip(car_cone_xs.flatten(), car_cone_ys.flatten())]
        min_x, min_y, max_x, max_y = polygon_calculate_bounding_box(cone_points)
        min_x, min_y, max_x, max_y = clip_bounding_box(min_x, min_y, max_x, max_y, self.obs_map.shape)
        roi = extract_roi(self.obs_map, min_x, min_y, max_x, max_y)
        adjusted_polygon = adjust_polygon_to_roi(cone_points, min_x, min_y)
        state.obs_ahead = check_polygon_intersection(roi, adjusted_polygon)
        return state.obs_ahead

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
            print(f"idx={state_idx}")
            if self.local_obs_detected(cur_state, cone_radius=SENSE_CONE_RADIUS, cone_fov=SENSE_CONE_ANGLE):
                self.mpc_mode = True
                self.mpc_curr_iteration = 0

            if self.mpc_mode and MPC_ENABLE:
                delta, _, closest_index = self.mpc.predict(cur_state, DELTA_T)
                # TODO: once finalized MPC w/ KRRT imp inside, we will replace the PP w/ it.
                # for now, we just want to statistics and plots over what it does
                pp_delta, target_ind, closest_index =\
                    self.lp.pure_pursuit_steer_control(cur_state, self.trajectory, DELTA_T)
                if delta is None:
                    delta = pp_delta
                    
                if (self.mpc_curr_iteration + 1) < MPC_ITEREATIONS:
                    self.mpc_curr_iteration += 1
                else:
                    self.mpc_mode = False
                    self.mpc_curr_iteration = 0
            else:
                delta, target_ind, closest_index = self.lp.pure_pursuit_steer_control(cur_state, self.trajectory, DELTA_T)

            if DEBUG_ENABLE: print(f"delta {delta}", end=" ")
            target_speed = TARGETED_SPEED
            target_speed_coef = 0.01
            target_speed_decrease_gain = 10
            target_speed_delta_diff = np.abs(delta) - target_speed_coef * MAX_DSTEER
            if  target_speed_delta_diff > 0:
                target_speed = max(0,TARGETED_SPEED - target_speed_decrease_gain*target_speed_delta_diff)
                if DEBUG_ENABLE: print(f"target_speed {target_speed}")

            cur_state.v = self.lp.proportional_control_acceleration(target_speed, cur_state.v, DELTA_T)
            cur_state.predelta = delta
            cur_state = self.sim.update_state(cur_state, delta)  # Control vehicle
            self.clock += DELTA_T
            cur_state.t = self.clock
            cur_state.a = delta
            self.states.append(cur_state)
            self.closest_path_coords.append([self.trajectory.cx[closest_index], self.trajectory.cy[closest_index]])
            self.target_path_coords.append([self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]])
            if DEBUG_ENABLE: print("")

