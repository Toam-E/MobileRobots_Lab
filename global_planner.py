import numpy as np
import matplotlib.pyplot as plt
from utils import CSpace
from kino_rrt import KINORRT
from plot_utils import Plotter, inflate
plt.ion()


def main():
    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    resolution=0.05000000074505806
    inflated_map = inflate(map_original, 0.2/resolution)
    converter = CSpace(resolution, origin_x=-4.73, origin_y=-5.66, map_shape=map_original.shape)
    start=converter.meter2pixel([0.0,0.0])
    goal = converter.meter2pixel([6.22, -4.22])
    print(start)
    print(goal)
    cost = None
    while cost is None:
        kinorrt_planner = KINORRT(env_map=inflated_map, max_step_size=20, max_itr=10000, p_bias=0.05,converter=converter )
        path, path_idx, cost = kinorrt_planner.find_path(start, goal)
        print(f'cost: {cost}')
        if cost != None:
            path_meter = np.array(converter.pathindex2pathmeter(path))
            np.save(f'krrt_path.npy', path_meter)
            np.save(f'krrt_path_idx.npy', path_idx)
    plotter = Plotter(inflated_map=inflated_map)
    plotter.draw_tree(kinorrt_planner.tree, start, goal, path, path_idx)
    


if __name__ == "__main__":
    main()


