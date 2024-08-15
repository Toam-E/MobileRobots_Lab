import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap, BoundaryNorm
from sim_state import SimState, SimStatesContainer
from consts import *
from trajectory import Trajectory
import imageio
from datetime import datetime
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
        states_mpc_local_goals = [ state.mpc_local_goal_pixel for state in states]
        states_mpc_bbox = [ state.mpc_bbox for state in states]
        states_mpc_krrt_paths = [ state.krrt_path for state in states]

        cmap = ListedColormap(['black', 'yellow', 'blue'])
        bounds = [0, 50, 150, 255]
        norm = BoundaryNorm(bounds, cmap.N)

        num_of_states = len(states) 
        for i in range(num_of_states):
            print(f"i={i+1}/{num_of_states}")

            fig, ax = plt.subplots(dpi=150)  # Create a new figure and axis
            #fig.patch.set_facecolor('gray')  # Set background color to gray
            #ax.set_facecolor('gray')  # Set the axes background color to gray
            ax.axis('off')
            #ax.imshow(self.map, origin="lower") #cmap=cmap, norm=norm, origin="lower")
            ax.imshow(self.map, cmap=cmap, norm=norm, origin="lower")

            plt.scatter(start[0], start[1], s=100, c='g')
            plt.scatter(goal[0],goal[1], s=100, c='r')

            if states_mpc_bbox[i] is not None:
                x_min, y_min, x_max, y_max = states_mpc_bbox[i]
                x_coords = [x_min, x_max, x_max, x_min]
                y_coords = [y_min, y_min, y_max, y_max]
                plt.fill(x_coords, y_coords, color='orange', alpha=0.5)

            if trajectory is not None:
                plt.plot(trajectory.cx, trajectory.cy, label="trajectory", linewidth=2, color='cyan')
            plt.plot(states_x[:i], states_y[:i], label="actual", linewidth=2, color='green')

            if states_mpc_krrt_paths[i] is not None:
                krrt_path_x = [ elem[0] for elem in states_mpc_krrt_paths[i]]
                krrt_path_y = [ elem[1] for elem in states_mpc_krrt_paths[i]]
                plt.plot(krrt_path_x, krrt_path_y, label="krrt", linewidth=1, color='red')
            #circle = patches.Circle((states.x[i], states.y[i]), lidar_range, edgecolor='blue', facecolor='none', linewidth=1)
            #ax.add_patch(circle)

            #self.plot_car(states.x[i], states.y[i], states.yaw[i], steer=states.d[i])
            if target_path_coords is not None:
                plt.scatter(target_path_coords[i][0], target_path_coords[i][1], c='w')
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

