import numpy as np
from planner import QuinticPolynomial

def make_trajectory():

    sx = 10.0  # start x position [m]
    sy = 10.0  # start y position [m]

    syaw = np.deg2rad(10.0)  # start yaw angle [rad]
    sv = 1.0  # start speed [m/s]
    sa = 0.1  # start accel [m/ss]
    gx = 30.0  # goal x position [m]
    gy = -10.0  # goal y position [m]
    gyaw = np.deg2rad(20.0)  # goal yaw angle [rad]
    gv = 1.0  # goal speed [m/s]
    ga = 0.1  # goal accel [m/ss]
    max_accel = 1.0  # max accel [m/ss]
    max_jerk = 0.5  # max jerk [m/sss]
    dt = 0.1  # time tick [s]
    

if __name__ == '__main__':
    pass