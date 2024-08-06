import numpy as np
import matplotlib.pyplot as plt
from utils import Tree

class Plotter():
    def __init__(self, map):
        self.env_rows, self.env_cols = map.shape
        self.map = map
    
    def draw_tree(self, tree:Tree, start, goal, path=None, path_idx=None):
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.xlim([0, self.env_cols])
        plt.ylim([0, self.env_rows])
        if path is not None:
            for idx in path_idx:
                try:
                    vertex = tree.vertices[idx]
                    for waypoint in vertex.waypoints:
                        plt.scatter(waypoint[0], waypoint[1], s=20, c='m')
                except:
                    pass

        #for i in range(len(tree.vertices)):
        #    conf = tree.vertices[i].conf
        #    plt.scatter(conf[0], conf[1], s=10, c='b')

        
        plt.scatter(start[0], start[1], s=100, c='g')
        plt.scatter(goal[0], goal[1], s=100, c='r')
        plt.imshow(self.map, origin="lower")
        #plt.pause(2)
 

def plot_map(map):
    plt.imshow(map, origin="lower")
    plt.pause(2)




