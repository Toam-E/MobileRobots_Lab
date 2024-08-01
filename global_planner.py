import numpy as np
import matplotlib.pyplot as plt
from py_Utils import Tree,  CSpace, calc_configs_dist
plt.ion()

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


class Plotter():
    def __init__(self, inflated_map): 
        self.env_rows, self.env_cols = inflated_map.shape
        self.map = inflated_map
    
    def draw_tree(self, tree:Tree, start, goal, path=None, path_idx = None):
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

        for i in range(len(tree.vertices)):
            conf = tree.vertices[i].conf
            plt.scatter(conf[0], conf[1], s=10, c='b')

        
        plt.scatter(start[0], start[1], s=100, c='g')
        plt.scatter(goal[0],goal[1], s=100, c='r')
        plt.imshow(self.map, origin="lower")
        plt.pause(100)
 

    


def inflate(map_, inflation):#, resolution, distance):
    cells_as_obstacle = int(inflation) #int(distance/resolution)
    map_[95:130, 70] = 100
    original_map = map_.copy()
    inflated_map = map_.copy()
    # add berrier
    rows, cols = inflated_map.shape
    for j in range(cols):
        for i in range(rows):
            if original_map[i,j] != 0:
                i_min = max(0, i-cells_as_obstacle)
                i_max = min(rows, i+cells_as_obstacle)
                j_min = max(0, j-cells_as_obstacle)
                j_max = min(cols, j+cells_as_obstacle)
                inflated_map[i_min:i_max, j_min:j_max] = 100
    return inflated_map       
    

class Odom(object):
    def __init__(self, converter:CSpace):
        self.wheelbase = 0.35
        self.max_steering_angle = np.deg2rad(35)
        self.min_velocity, self.max_velocity = 0.5, 1
        self.min_time, self.max_time = 1, 2
        self.converter = converter
    
    def sample_control_command(self):
        delta_time = np.random.uniform(self.min_time, self.max_time)
        steering = np.random.uniform(-self.max_steering_angle, self.max_steering_angle)
        velocity = np.random.uniform(self.min_velocity, self.max_velocity)
        return delta_time, steering, velocity

    def propagate(self,  steering, velocity ,delta_time, initial_x):
        initial_x = self.converter.pixel2meter(initial_x)
        x = initial_x[0]
        y = initial_x[1]
        theta= initial_x[2]
        theta_dot = velocity * np.tan(steering) / self.wheelbase
        dt = 0.03
        edge = [[x,y,theta]]
        cost = 0
        for _ in range(int(delta_time/dt)):
            theta += theta_dot * dt
            x_dot = velocity * np.cos(theta)
            y_dot = velocity * np.sin(theta)
            x += x_dot * dt
            y += y_dot * dt
            cost += ((edge[-1][0] - x)**2 + (edge[-1][1] - y)**2)**0.5
            edge.append([x,y,theta])
        edge = self.converter.pathmeter2pathindex(edge)
        new_state = edge[-1]
        return new_state, edge, cost




def main():
    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    resolution=0.05000000074505806
    inflated_map = inflate(map_original, 0.2/resolution)
    converter = CSpace(resolution, origin_x=-4.73, origin_y=-5.66, map_shape=map_original.shape)
    start=converter.meter2pixel([0.0,0.0])
    goal = converter.meter2pixel([6.22, -4.22])
    print(start)
    print(goal)
    kinorrt_planner = KINORRT(env_map=inflated_map, max_step_size=20, max_itr=10000, p_bias=0.05,converter=converter )
    path, path_idx, cost = kinorrt_planner.find_path(start, goal)
    print(f'cost: {cost}')
    if cost != None:
        path_meter = np.array(converter.pathindex2pathmeter(path))
        np.save(f'krrt_path.npy', path_meter)    
    plotter = Plotter(inflated_map=inflated_map)
    plotter.draw_tree(kinorrt_planner.tree, start, goal, path, path_idx)
    


if __name__ == "__main__":
    main()


