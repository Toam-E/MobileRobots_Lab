from state import State
import numpy as np
from cspace import CSpace
from utils import get_normalized_angle

class SimState(State):
    """
    Simulation state class
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super().__init__(x, y, yaw, v)
        self.t = None
        self.d = None
        self.a = None
        self.cone = None
        self.obs_ahead = None
        self.mpc_local_goal = None
        self.mpc_local_goal_pixel = None
        self.mpc_bbox = None

    def reset_mpc_related(self):
        self.mpc_local_goal = None
        self.mpc_local_goal_pixel = None
        self.mpc_bbox = None


class SimStatesContainer(object):
    def __init__(self):
        self.states = []
        self.index = 0

    def __iter__(self):
        self.index = 0
        return self

    def __next__(self):
        if self.index < len(self.states):
            result = self.states[self.index]
            self.index += 1
            return result
        else:
            raise StopIteration        

    def __len__(self):
        return len(self.states)
    
    def append(self, state: SimState):
        self.states.append(state)

    def get_states_in_pixels(self, converter:CSpace):
        states = SimStatesContainer()
        for idx, state in enumerate(self.states):
            state_pixel_list = converter.meter2pixel([state.x, state.y, state.yaw])
            state_pixel = SimState(x=state_pixel_list[0], y=state_pixel_list[1],\
                                yaw=state_pixel_list[2], v=state.v)
            state_pixel.obs_ahead = state.obs_ahead
            state_pixel.mpc_local_goal_pixel = state.mpc_local_goal_pixel
            state_pixel.mpc_bbox = state.mpc_bbox
            states.append(state_pixel)
        return states
    
    def calc_states_cones(self, cone_radius = 15, cone_fov=np.pi/3):
        # add "visibility cone" to demonstrate what the car sees

        # compute a pixeled arc for the cone
        for state in self.states:
            cone_origin_x = state.x
            cone_origin_y = state.y
            cone_origin_yaw = state.yaw
            fov_angles = np.linspace(start=cone_fov/2, stop=-cone_fov/2, num=cone_radius)
            tile_yaw = np.tile(cone_origin_yaw, fov_angles.size)
            fov_angles = np.expand_dims(tile_yaw + fov_angles, axis=0)
            car_angles = np.apply_along_axis(get_normalized_angle, 0, fov_angles)
            car_cone_xs = cone_origin_x + cone_radius * np.cos(car_angles)
            car_cone_ys = cone_origin_y + cone_radius * np.sin(car_angles)

            car_cone_xs = np.append(np.insert(car_cone_xs, 0, cone_origin_x), cone_origin_x)
            car_cone_ys = np.append(np.insert(car_cone_ys, 0, cone_origin_y), cone_origin_y)

            state.cone = [car_cone_xs, car_cone_ys]
        return
