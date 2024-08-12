import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
from consts import *
from cspace import CSpace
from trajectory import Trajectory
# import  rc_car.scripts.utils.cubic_spline_planner as cubic_spline_planner
import imageio
from datetime import datetime
from utils import get_normalized_angle
#from PIL import Image

MAX_TIME = 500.0  # max simulation time
TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
# DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 0.5  # [m]
WIDTH = car_consts.car_width  # [m]
BACKTOWHEEL = 0.1  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.07  # [m]
TREAD = 0.2  # [m]



class State:
    """
    vehicle state class
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.predelta = 0.0

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
        self.mpc_goal = None

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
            if state.mpc_goal is not None:
                state_pixel.mpc_goal = converter.meter2pixel(state.mpc_goal)
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

class Simulator(object):
    def __init__(self, map,  trajectory=None ,DT = 0.1):
        self.trajectory = trajectory
        self.time = 0.0
        self.dt = DT
        self.map=map

    def plot_car(self, x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                            [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])
        rr_wheel = np.copy(fr_wheel)
        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                        [-math.sin(steer), math.cos(steer)]])
        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB
        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T
        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        plt.plot(np.array(outline[0, :]).flatten(),
                np.array(outline[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fr_wheel[0, :]).flatten(),
                np.array(fr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rr_wheel[0, :]).flatten(),
                np.array(rr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fl_wheel[0, :]).flatten(),
                np.array(fl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rl_wheel[0, :]).flatten(),
                np.array(rl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(x, y, "*")


    def update_state(self, state: SimState, delta):
        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER
        state.x = state.x + state.v * math.cos(state.yaw) * self.dt
        state.y = state.y + state.v * math.sin(state.yaw) * self.dt
        state.yaw = state.yaw + state.v / WB * math.tan(delta) * self.dt
        # state.v = state.v + a * self.dt
        if state.v > MAX_SPEED:
            state.v = MAX_SPEED
        elif state.v < MIN_SPEED:
            state.v = MIN_SPEED
        state.rear_x = state.x - ((WB / 2) * math.cos(state.yaw))
        state.rear_y = state.y - ((WB / 2) * math.sin(state.yaw))
        self.time +=  self.dt
        return state

    def smooth_yaw(self, yaw):
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]
            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]
            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]
        return yaw
    

    def create_animation(self, start, goal, states: SimStatesContainer,\
                         trajectory: Trajectory = None, target_path_coords=None,\
                         closest_path_coords=None, fps=15):

        sim_plan = []

        states_x = [ state.x for state in states]
        states_y = [ state.y for state in states]
        states_cone = [ state.cone for state in states]
        states_obs_ahead = [ state.obs_ahead for state in states]
        states_mpc_local_goals = [ state.mpc_goal for state in states]

        num_of_states = len(states) 
        for i in range(num_of_states):
            print(f"i={i+1}/{num_of_states}")

            fig, ax = plt.subplots(dpi=150)  # Create a new figure and axis
            ax.axis('off')
            ax.imshow(self.map, origin="lower")

            plt.scatter(start[0], start[1], s=100, c='g')
            plt.scatter(goal[0],goal[1], s=100, c='r')

            if trajectory is not None:
                plt.plot(trajectory.cx, trajectory.cy, label="trajectory", linewidth=2, color='cyan')
            plt.plot(states_x[:i], states_y[:i], label="actual", linewidth=2, color='green')
            #circle = patches.Circle((states.x[i], states.y[i]), lidar_range, edgecolor='blue', facecolor='none', linewidth=1)
            #ax.add_patch(circle)

            #self.plot_car(states.x[i], states.y[i], states.yaw[i], steer=states.d[i])
            if target_path_coords is not None:
                plt.scatter(target_path_coords[i][0], target_path_coords[i][1], c='b')
            if closest_path_coords is not None:
                plt.scatter(closest_path_coords[i][0], closest_path_coords[i][1])

            if states_obs_ahead[i] is not None:
                if states_obs_ahead[i]:
                    cone_color = "red"
                else:
                    cone_color = "mediumpurple"
                plt.fill(states_cone[i][0], states_cone[i][1], cone_color, zorder=13, alpha=0.5)

            if states_mpc_local_goals[i] is not None:
                plt.scatter(states_mpc_local_goals[i][0], states_mpc_local_goals[i][1],s=20, c='r')


            ax.grid(False)
            plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
            #plt.title("Time[s]:" + str(round(states.t[i], 2))
            #        + ", speed[m/s]:" + str(round(states.v[i], 2)))
            #plt.pause(0.00001)


            # convert plot to image
            canvas = plt.gca().figure.canvas
            canvas.draw()
            data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            data = data.reshape(canvas.get_width_height()[::-1] + (3,))
            #pic_time = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
            #imageio.imwrite(f"out/{i}_{pic_time}.jpeg", data)
            # Resize the image before appending it to the list
            
            #data = resize_image(data, new_width, new_height)
            #print(data.shape)
            x_min, x_max, y_min, y_max = (0, data.shape[0], int(0.3*data.shape[0]), int(0.7*data.shape[0]))
            data = data[y_min:y_max, x_min:x_max]

            sim_plan.append(data)

            # Close the figure to release memory
            plt.close(fig)

        # store gif
        plan_time = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        imageio.mimwrite(f'plan_{plan_time}.gif', sim_plan, 'GIF', fps=fps, subrectangles=True)

    def save_simulation(self):
        pass
    


"""
def resize_image(image, new_width, new_height):
    pil_image = Image.fromarray(image)
    resized_pil_image = pil_image.resize((new_width, new_height), Image.ANTIALIAS)
    resized_image = np.array(resized_pil_image)
    return resized_image
"""
