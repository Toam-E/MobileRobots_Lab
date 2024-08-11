import car_consts


#  hyper-parameters
LF_K = 0.1  # look forward gain
LFC = 1.0  # [m] look-ahead distance
V_KP = 1.0  # speed proportional gain
DELTA_T = 0.1  # [s] time tick
TARGETED_SPEED = 1.0  # [m/s]
T = 100.0  # max simulation time
WB = car_consts.wheelbase 
MAX_STEER = car_consts.max_steering_angle_rad  # maximum steering angle [rad]
MAX_DSTEER = car_consts.max_dt_steering_angle  # maximum steering speed [rad/s]
MAX_SPEED = car_consts.max_linear_velocity  # maximum speed [m/s]
MIN_SPEED = car_consts.min_linear_velocity  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]


# Kino RRT params
KINORRT_TRIES           = 4
KINORRT_ITER            = 50 #350
KINORRT_MAX_STEP_SIZE   = 25


NEXT_IDX = 20

resolution = 0.05000000074505806
inflation = 0.2/resolution