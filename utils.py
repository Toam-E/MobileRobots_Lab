#!/usr/bin/env python3

import numpy as np
import math
import bisect
import operator



def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q



class CubicSpline1D:
    """
    1D Cubic Spline class

    Parameters
    ----------
    x : list
        x coordinates for data points. This x coordinates must be
        sorted
        in ascending order.
    y : list
        y coordinates for data points
    """

    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        """
        Calc `y` position for given `x`.

        if `x` is outside the data point's `x` range, return None.

        Returns
        -------
        y : float
            y position for given x.
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def calc_first_derivative(self, x):
        """
        Calc first derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        dy : float
            first derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, x):
        """
        Calc second derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        ddy : float
            second derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    """
    Cubic CubicSpline2D class

    Parameters
    ----------
    x : list
        x coordinates for data points.
    y : list
        y coordinates for data points.

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        k : float
            curvature for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


class Tree(object):
    
    def __init__(self):
        self.vertices = dict()
        self.edges = dict()

    def GetRootID(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def GetNearestVertex(self, config):
        '''
        Returns the nearest state ID in the tree.
        @param config Sampled configuration.
        '''
        dists = []
        for _, v in self.vertices.items():
            dists.append(calc_configs_dist(config, v.conf))

        vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid].conf
            
    def GetKNN(self, config, k):
        '''
        Return k-nearest neighbors
        @param config Sampled configuration.
        @param k Number of nearest neighbors to retrieve.
        '''
        dists = []
        for _, v in self.vertices.items():
            dists.append(calc_configs_dist(config, v.conf))

        dists = np.array(dists)
        knnIDs = np.argpartition(dists, k)[:k]
        # knnDists = [dists[i] for i in knnIDs]

        return knnIDs #, [self.vertices[vid] for vid in knnIDs]

    def AddVertex(self, config):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices[vid] = RRTVertex(conf=config)
        return vid

    def AddEdge(self, sid, eid, arc_cost=None):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid
        if arc_cost is None:
            edge_cost = calc_configs_dist(self.vertices[sid].conf, self.vertices[eid].conf)
        else:
            edge_cost = arc_cost
        self.vertices[eid].set_cost(cost=self.vertices[sid].cost + edge_cost)

    def getIndexForState(self, conf):
        '''
        Search for the vertex with the given configuration and return the index if exists
        @param conf configuration to check if exists.
        '''
        valid_idxs = [v_idx for v_idx, v in self.vertices.items() if (v.conf[:2] == conf[:2]).all()]
        if len(valid_idxs) > 1:
            print('multiple goals')
        if len(valid_idxs) > 0:
            return valid_idxs[0]
        return None

    def isConfExists(self, conf):
        '''
        Check if the given configuration exists.
        @param conf configuration to check if exists.
        '''
        conf_idx = self.getIndexForState(conf=conf)
        if conf_idx is not None:
            return True
        return False
'''
    def save_tree(self, file_path):
        with open(file_path, 'wb') as f:
            pickle.dump(self, f)

    @staticmethod
    def load_tree(file_path):
        with open(file_path, 'rb') as f:
            return pickle.load(f)
'''


class RRTVertex(object):
    def __init__(self, conf, cost=0):
        self.conf = conf
        self.cost = cost

    def set_cost(self, cost):
        self.cost = cost

    def set_waypoints(self, waypoints):
        self.waypoints = np.array(waypoints)
    


def calc_configs_dist(config1, config2):
    config1 = np.array(config1)
    config2 = np.array(config2)
    return np.linalg.norm(config1[:2] - config2[:2], ord=2)


class Ackermann_ARC(object):
    def __init__(self, wheelbase = 0.35, dt=0.03):
        self.wheelbase = wheelbase
        self.dt = dt

    def get_arc(self,x_near, x_new ,velocity):
        # print('in')
        # print(f'xnew {x_new} xnear {x_near}')
        Lf = ((x_new[0]-x_near[0])**2 + (x_new[1]-x_near[1])**2)**0.5
        # print(f'lf {Lf}')
        alpha = math.atan2(x_new[1] - x_near[1], x_new[0] - x_near[0]) - x_near[2]
        # print(f'alpha {alpha}')
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha) / Lf, 1.0)
        # print(f'steering {steering}')
        theta_dot = velocity * np.tan(steering) / self.wheelbase
        reach_goal= False
        total_time = 0
        x, y, yaw = x_near
        yaw = self.fix_yaw_range(yaw)
        waypoints = []
        waypoints.append([x, y, yaw])
        while not reach_goal:
            total_time += self.dt
            yaw += theta_dot * self.dt
            yaw = self.fix_yaw_range(yaw)
            x_dot = velocity * np.cos(yaw)
            y_dot = velocity * np.sin(yaw)
            x += x_dot * self.dt
            y += y_dot * self.dt
            waypoints.append([x, y, yaw])
            dist2goal = ((x - x_new[0])**2 + (y - x_new[1])**2)**0.5
            if dist2goal < 0.05:
                reach_goal = True
            # print('in loop')
        length = 0.0
        for idx in range(len(waypoints)-1):
            length += np.linalg.norm(np.array(waypoints[idx][:2]) - np.array(waypoints[idx+1][:2]))
        # print('out')
        return np.array(waypoints), total_time, length, waypoints[-1][2]

    def fix_yaw_range(self, yaw):
        if yaw > np.pi*2:
            yaw = yaw - np.pi*2
        if yaw < np.pi*2:
            yaw = yaw + np.pi*2
        return yaw



