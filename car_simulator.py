import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import math
import numpy as np
import car_consts
from utils import CSpace
# import  rc_car.scripts.utils.cubic_spline_planner as cubic_spline_planner
import imageio
from datetime import datetime


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
WB = car_consts.wheelbase  # [m]

MAX_STEER = car_consts.max_steering_angle_rad  # maximum steering angle [rad]
MAX_DSTEER = car_consts.max_dt_steering_angle  # maximum steering speed [rad/s]
MAX_SPEED = car_consts.max_linear_velocity  # maximum speed [m/s]
MIN_SPEED = car_consts.min_linear_velocity  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True

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
    

class States(object):
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.d = []
        self.a = []
        self.rear_x = []
        self.rear_y = []

    def append(self, t, state: State,a=0, delta=0 ):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)
        self.d.append(delta)
        self.a.append(a)
        self.rear_x.append(state.rear_x)
        self.rear_y.append(state.rear_y)

    def get_states_in_meters(self, converter:CSpace):
        states = States()
        for state in zip(self.x, self.y):
            state_pixel = converter.meter2pixel([state[0], state[1]])
            states.x.append(state_pixel[0])
            states.y.append(state_pixel[1])
        return states

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


    def update_state(self, state: State, delta):
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
    

    def show_simulation(self, states: States, closest_path_coords=None):
        for i in range(len(states.x)-1):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            if self.trajectory is not None:
                plt.scatter(self.trajectory.cx, self.trajectory.cy, c='cyan')#, "-r", label="course")
                # plt.plot(self.trajectory.ax, self.trajectory.ay, "-b", label="course")
            plt.plot(states.x[:i], states.y[:i], label="trajectory", linewidth=2, color='green')
            #self.plot_car(states.x[i], states.y[i], states.yaw[i], steer=states.d[i])
            if closest_path_coords is not None:
                plt.scatter(closest_path_coords[i][0], closest_path_coords[i][1])
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(states.t[i], 2))
                    + ", speed[m/s]:" + str(round(states.v[i], 2)))
            plt.pause(0.00001)

    def create_map_visualization(self, ax, map_array):
        """
        Visualize the map on the given axis.
        @param ax The axis to plot on.
        @param map_array The NumPy array representing the map.
        """
        ax.imshow(map_array, origin="lower")

    def create_animation(self, states: States, converter:CSpace, start, goal, closest_path_coords=None):
         # switch backend - possible bugfix if animation fails
        #matplotlib.use('TkAgg')

        sim_plan = []
        traj_meters_cx = []
        traj_meters_cy = []
        for traj_point in zip(self.trajectory.cx, self.trajectory.cy):
            traj_pixel = converter.meter2pixel([traj_point[0], traj_point[1]])
            traj_meters_cx.append(traj_pixel[0])
            traj_meters_cy.append(traj_pixel[1])

        for i in range(len(states.x)):
            print(f"i={i+1}/{len(states.x)}")

            fig, ax = plt.subplots(dpi=300)  # Create a new figure and axis
            ax.axis('off')
            self.create_map_visualization(ax, self.map)  # Pass the axis to your visualization function            

            plt.scatter(start[0], start[1], s=100, c='g')
            plt.scatter(goal[0],goal[1], s=100, c='r')

            # for stopping simulation with the esc key.
            #plt.gcf().canvas.mpl_connect('key_release_event',
            #        lambda event: [exit(0) if event.key == 'escape' else None])
            if self.trajectory is not None:
                plt.scatter(traj_meters_cx, traj_meters_cy, c='cyan')#, "-r", label="course")
                # plt.plot(self.trajectory.ax, self.trajectory.ay, "-b", label="course")
            plt.plot(states.x[:i], states.y[:i], label="trajectory", linewidth=2, color='green')
            #self.plot_car(states.x[i], states.y[i], states.yaw[i], steer=states.d[i])
            #if closest_path_coords is not None:
            #    plt.scatter(closest_path_coords[i][0], closest_path_coords[i][1])
            #plt.axis("equal")
            #plt.grid(True)
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
            sim_plan.append(data)

            # Close the figure to release memory
            plt.close(fig)

        # store gif
        plan_time = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        imageio.mimwrite(f'plan_{plan_time}.gif', sim_plan, 'GIF', duration=0.05, subrectangles=True)

    def save_simulation(self):
        pass
    

