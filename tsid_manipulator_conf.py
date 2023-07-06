import numpy as np

N_SIMULATION = 1000  # number of time steps simulated
WITH_ORI = 0

dt = 0.001  # controller time step
q0 = np.array([0., 0, 1.57])  # initial configuration

w_ee = 1.0  # weight of end-effector task
w_posture = 1e-3  # weight of joint posture task
w_torque_bounds = 1.0  # weight of the torque bounds
w_joint_bounds = 1.0  # weight of the joint bounds

kp_ee = 1000.0  # proportional gain of end-effector constraint
kp_posture = 100.0  # proportional gain of joint posture task

tau_max_scaling = 1.  # scaling factor of torque bounds
v_max_scaling = 1.  # scaling factor of velocity bounds

ee_frame_name = "end_link"  # end-effector frame name
ee_task_mask = np.array([1, 1, 1, 1, 1, 1]) if WITH_ORI else np.array([1, 1, 1, 0, 0, 0])
ee_task_local_frame = False  # specifies whether task is formulated in local frame

PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 20  # update robot configuration in viwewer every DISPLAY_N time steps

from os.path import join

urdf = "three_link.urdf"
path = ""
urdf = join(path, urdf)
path = join(path, "../..")
