import numpy as np
from planner import quintic_polynomials_planner


from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC

WORKSPACE_WALLS = np.array([
        # sides
        [0.15, 0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
        [0.15, -0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
        # back
        [-0.41, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
        # front
        [0.75, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
        # top
        [0.2, 0, 1, 0, 0, 0, 1.2, 1, 0.01],
        # bottom
        [0.2, 0, -0.05, 0, 0, 0, 1.2, 1, 0.01]
    ])

def main():
    trajectory_ = []

    franka_arm = FrankaArm()

    sy = franka_arm.translation[1]

    ########################################### ACCELERATION ###########################################
    sx = 10  # start x position [m]
    sz = 10  # start y position [m]

    # sx = franka_arm.translation[0]  # start x position [m]
    # sz = franka_arm.translation[2]  # start y position [m]

    syaw = np.deg2rad(0.0)  # start yaw angle [rad]
    sv = 1.0  # start speed [m/s]
    sa = 0.1  # start accel [m/ss]

    #! TRAJEC VARS
    gx = 30.0  # goal x position [m]
    gz = 10.0  # goal y position [m]
    gyaw = np.deg2rad(0.0)  # goal yaw angle [rad]
    gv = 1.0  # goal speed [m/s]
    ga = 0.1  # goal accel [m/ss]


    max_accel = 1.0  # max accel [m/ss]
    max_jerk = 0.5  # max jerk [m/sss]
    dt = 0.1  # time tick [s]


    time, x, y, yaw, v, a, j = quintic_polynomials_planner(sx, sz, syaw, sv, sa, gx, gz, gyaw, gv, ga, max_accel, max_jerk, dt)

    for x_val, y_val, t in zip(x, y, time):
        trajectory_point = RigidTransform(translation=np.array([sx, sz, sz]), rotation=franka_arm.rotation, from_frame=franka_arm.from_frame, to_frame=franka_arm.from_frame)
        trajectory_.append(trajectory_point)

    ########################################### DE-ACCELERATION ###########################################
    sx = gx  # start x position [m]
    sz = gz  # start y position [m]
    syaw = gyaw  # start yaw angle [rad]
    sv = gv  # start speed [m/s]
    sa = ga  # start accel [m/ss]

    #! Final points
    gx = 30.0  # goal x position [m]
    gz = 10.0  # goal y position [m]
    gyaw = np.deg2rad(0.0)  # goal yaw angle [rad]
    gv = 1.0  # goal speed [m/s]
    ga = 0.1  # goal accel [m/ss]
    max_accel = 1.0  # max accel [m/ss]
    max_jerk = 0.5  # max jerk [m/sss]
    dt = 0.1  # time tick [s]


    time, x, y, yaw, v, a, j = quintic_polynomials_planner(sx, sz, syaw, sv, sa, gx, gz, gyaw, gv, ga, max_accel, max_jerk, dt)

    for x_val, y_val, t in zip(x, y, time):
        trajectory_point = RigidTransform(translation=np.array([sx, sz, sz]), rotation=franka_arm.rotation, from_frame=franka_arm.from_frame, to_frame=franka_arm.from_frame)
        trajectory_.append(trajectory_point)
    
    collision_check(trajectory_)

def collision_check(trajectory):

    for pose in trajectory:
        x = pose.translation[0]
        y = pose.translation[1]
        z = pose.translation[2]

        # bottom
        if(z<WORKSPACE_WALLS[5][2]):
            return True   
        # front
        elif(x>WORKSPACE_WALLS[3][0]):
            return True
        #top
        elif(z>WORKSPACE_WALLS[4][2]):
            return True
    return False


########################################### MAIN ###########################################
if __name__ == '__main__':
    main()