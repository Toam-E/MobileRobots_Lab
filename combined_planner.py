import copy
from trajectory import Trajectory
from consts import *
from kino_rrt import KINORRT
from car_simulator import SimState, SimStatesContainer, Simulator
from local_planner import LocalPlanner

# run params
RUN_KRRT = False
RUN_ADD_OBS = True
RUN_COMBINED_PLANNER = True
RUN_ANIMATION = True

class CombinedPlanner(object):

    def __init__(self, trajectory : Trajectory, sim : Simulator, states : SimStatesContainer,\
                 local_planner : LocalPlanner):
        self.trajectory = trajectory
        self.sim = sim
        self.states = states
        self.lp = local_planner
        self.lastIndex = len(trajectory.cx) - 1
        self.clock = 0.0
        self.target_path_coords = []
        self.closest_path_coords = []

    def find_path(self):
        cur_state = SimState(x=self.trajectory.cx[0], y=self.trajectory.cy[0], yaw=self.trajectory.cyaw[0], v=0.0)
        cur_state.t = self.clock
        self.states.append(cur_state)
        target_ind, _, _ = self.lp.search_target_index(cur_state)
        self.target_path_coords.append([self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]])
        self.closest_path_coords.append([self.trajectory.cx[0], self.trajectory.cy[0]])

        while T >= self.clock and self.lastIndex > target_ind:
            cur_state = copy.copy(cur_state)
            if self.lp.local_obs_detected(cur_state, cone_radius=SENSE_CONE_RADIUS, cone_fov=SENSE_CONE_ANGLE):
                # run KRRT locally and find a new route
                pass

            cur_state.v = self.lp.proportional_control_acceleration(TARGETED_SPEED, cur_state.v, DELTA_T)
            delta, target_ind, closest_index = self.lp.pure_pursuit_steer_control(cur_state, self.trajectory, DELTA_T)
            cur_state.predelta = delta
            cur_state = self.sim.update_state(cur_state, delta)  # Control vehicle
            self.clock += DELTA_T
            cur_state.t = self.clock
            cur_state.a = delta
            self.states.append(cur_state)
            self.closest_path_coords.append([self.trajectory.cx[closest_index], self.trajectory.cy[closest_index]])
            self.target_path_coords.append([self.trajectory.cx[target_ind], self.trajectory.cy[target_ind]])

