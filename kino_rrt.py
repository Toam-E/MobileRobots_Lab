import numpy as np
import matplotlib.pyplot as plt
from utils import Tree,  calc_configs_dist
from cspace import CSpace
from odom import Odom

class KINORRT(object):
    def __init__(self, env_map, max_step_size = 0.5, max_itr=5000, p_bias = 0.05, converter: CSpace =None ):
        self.max_step_size = max_step_size
        self.max_itr = max_itr
        self.p_bias = p_bias
        self.tree = Tree()
        self.map = env_map
        self.env_rows, self.env_cols = env_map.shape
        self.env_yaw_range = 2*np.pi
        self.converter = converter
        self.ackerman = Odom(converter)
        self.goal_radius = 4

    def find_path(self, start, goal):
        itr = 0
        self.tree.AddVertex(start)

        while itr < self.max_itr:
            
            # sample random vertex
            x_random = self.sample(goal)

            # find nearest neighbor
            x_near_idx, x_near = self.tree.GetNearestVertex(x_random)
            
            # sample random control command
            delta_time, steering, velocity = self.ackerman.sample_control_command()
            
            # propagate
            x_new, edge, edge_cost = self.ackerman.propagate(steering, velocity ,delta_time, x_near)
            
            # add vertex and edge
            if self.local_planner(edge):
                x_new_idx = self.tree.AddVertex(x_new)
                self.tree.AddEdge(sid=x_near_idx, eid=x_new_idx, arc_cost=edge_cost)
                self.tree.vertices[x_new_idx].set_waypoints(edge)

                if calc_configs_dist(goal, x_new) < self.goal_radius:
                    return self.get_shortest_path(x_new_idx)

            itr += 1
            if itr%1000 ==0:
                print(f'itr: {itr}')
        return None, None, None
    
    def sample(self, goal):
        
        if np.random.rand() > self.p_bias:
            x = np.random.randint(0, self.env_cols-1)
            y = np.random.randint(0, self.env_rows-1)
            theta = np.random.uniform(0, self.env_yaw_range)
            rand_state = [x, y, theta]
        else:
            rand_state = goal
        return rand_state

    
    def is_in_collision(self, x_new):
        x, y = x_new[:2]
        x_int = int(x)
        y_int = int(y)
        if x_int < 0 or (x_int+1) >= self.env_cols or y_int < 0 or (y_int+1) >= self.env_rows\
            or self.map[y_int, x_int] != 0 or self.map[y_int+1, x_int] != 0\
            or self.map[y_int, x_int+1] != 0 or self.map[y_int+1, x_int+1] != 0:
            return True
        return False
    
    def local_planner(self, edge):
        for point in edge:
            if self.is_in_collision(point):
                return False
        return True

    def get_shortest_path(self, goal_idx):
        '''
        Returns the path and cost from some vertex to Tree's root
        @param dest - the id of some vertex
        return the shortest path and the cost
        '''
        cost = self.tree.vertices[goal_idx].cost

        path = []
        path_idx = []
        curr_idx = goal_idx
        while curr_idx != self.tree.GetRootID():
            curr_vertex = self.tree.vertices[curr_idx]
            path.append(curr_vertex.conf)
            path_idx.append(curr_idx)
            # we get start start idx (sid) from the edges as the 
            # key of and edge is the end idx (eid) and the value is the start idx (sid)
            curr_idx = self.tree.edges[curr_idx]

        # we add the root to the path
        path.append(self.tree.vertices[curr_idx].conf)
        path_idx.append(curr_idx)
        path.reverse()
        path_idx.reverse()
        return path, path_idx , cost


