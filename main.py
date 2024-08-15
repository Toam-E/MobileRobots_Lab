import numpy as np
from trajectory import Trajectory
from cspace import CSpace
from consts import *
from kino_rrt import KINORRT
from car_simulator import SimStatesContainer, Simulator
from utils import inflate, add_new_obstacles
from combined_planner import CombinedPlanner

COMBINED_CONT = False
if COMBINED_CONT:
    from combined_controller import CombinedController

# run params
RUN_KRRT = False
RUN_ADD_OBS = True
RUN_COMBINED_PLANNER = True
RUN_ANIMATION = True

def main():

    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    inflated_map = inflate(map_original, INFLATION)
    converter = CSpace(RESOLUTION, origin_x=-4.73, origin_y=-5.66, map_shape=map_original.shape)
    start = [0.0,0.0]
    start_pixel = converter.meter2pixel(start)
    goal = [6.22, -4.22]
    goal_pixel = converter.meter2pixel(goal)

    if RUN_KRRT:
        cost = None
        while cost is None:
            kinorrt_planner = KINORRT(env_map=inflated_map, max_itr=10000, p_bias=0.05, converter=converter)
            path, _, cost = kinorrt_planner.find_path(start_pixel, goal_pixel)
            print(f'cost: {cost}')
            if cost != None:
                path_meter = np.array(converter.pathindex2pathmeter(path))
                np.save(f'krrt_path_pixels_2.npy', path)
                np.save(f'krrt_path_meters_2.npy', path_meter)
    else:
        path = np.load('krrt_path_pixels_2.npy')
        path_meter = np.load('krrt_path_meters_2.npy')

    if RUN_ADD_OBS:
        # add on path new obstacles
        path_fractions = [0.2, 0.4, 0.6, 0.8]
        inflations = [4, 6, 8, 6]
        new_obs_map = add_new_obstacles(inflated_map, path, path_fractions, inflations)
        #plt.imshow(new_obs_map, origin="lower")
        #plt.show()
    else:
        new_obs_map = inflated_map

    trajectory = Trajectory(dl=0.1, path=path_meter, TARGET_SPEED=TARGETED_SPEED)
    simulator = Simulator(new_obs_map, trajectory, DELTA_T)
    sim_states_container = SimStatesContainer()

    if RUN_COMBINED_PLANNER:
        if COMBINED_CONT:
            combined_controller = CombinedController(path_meter, new_obs_map, converter)
            combined_controller.find_path(goal_pixel)
        else:
            combined_controller = CombinedPlanner(new_obs_map, \
                trajectory, converter, simulator, sim_states_container)
            combined_controller.find_path()

    if RUN_ANIMATION:
        states_pixels = sim_states_container.get_states_in_pixels(converter)
        traj_pixels = trajectory.get_trajectory_in_pixels(converter)
        target_path_coords_pixels = converter.pathmeter2pathindex(combined_controller.target_path_coords)
        #closest_path_coords_pixels = converter.pathmeter2pathindex(combined_controller.closest_path_coords)
        states_pixels.calc_states_cones(cone_radius=SENSE_CONE_RADIUS, cone_fov=SENSE_CONE_ANGLE)

        #simulator.show_simulation(states, closest_path_coords)
        simulator.create_animation(start_pixel, goal_pixel,\
                                states_pixels, traj_pixels,\
                                target_path_coords=target_path_coords_pixels,\
                                closest_path_coords=None, fps=15)

if __name__ == '__main__':
    main()
