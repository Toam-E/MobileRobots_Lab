import copy
from trajectory import Trajectory
from consts import *
from car_simulator import SimState, SimStatesContainer, Simulator
from pure_pursuit import PurePursuitController
from model_predictive_controller import ModelPredictiveController
from cspace import CSpace

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

