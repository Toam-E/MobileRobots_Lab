import math
import numpy as np
from utils import calc_spline_course
from cspace import CSpace

class Trajectory(object):
    def __init__(self, path ,dl=1.0, TARGET_SPEED=10.0 / 3.6 ):
        self.path = path
        self.dl = dl
        self.target_speed = TARGET_SPEED
        self.ax = []
        self.ay = []
        for coords in path:
            self.ax.append(coords[0])
            self.ay.append(coords[1])
      
        self.cx, self.cy, cyaw, self.ck, self.s = calc_spline_course(self.ax, self.ay, ds=dl)
        self.cyaw = self.smooth_yaw(cyaw)
        self.sp = self.calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED)
    
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


    def calc_speed_profile(self, cx, cy, cyaw, target_speed):
        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward
        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]
            move_direction = math.atan2(dy, dx)
            if dx != 0.0 and dy != 0.0:
                dangle = abs(self.pi_2_pi(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0
            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed
        speed_profile[-1] = 0.0
        return speed_profile
    
    def pi_2_pi(self, angle):
        return self.angle_mod(angle)
    
    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        """
        Angle modulo operation
        Default angle modulo range is [-pi, pi)

        Parameters
        ----------
        x : float or array_like
            A angle or an array of angles. This array is flattened for
            the calculation. When an angle is provided, a float angle is returned.
        zero_2_2pi : bool, optional
            Change angle modulo range to [0, 2pi)
            Default is False.
        degree : bool, optional
            If True, then the given angles are assumed to be in degrees.
            Default is False.

        Returns
        -------
        ret : float or ndarray
            an angle or an array of modulated angle.
        """
        if isinstance(x, float):
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle

    def get_trajectory_in_meters(self, converter:CSpace):
        traj = Trajectory(self.path, self.dl, self.target_speed)
        traj.cx = []
        traj.cy = []
        for traj_point in zip(self.cx, self.cy):
            traj_pixel = converter.meter2pixel([traj_point[0], traj_point[1]])
            traj.cx.append(traj_pixel[0])
            traj.cy.append(traj_pixel[1])
        return traj
