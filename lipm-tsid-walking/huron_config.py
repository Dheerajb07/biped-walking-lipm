# Configuration file for HURON

import os
import numpy as np
import pinocchio as pin

# Display params
np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60
PRINT_N = 50           # print every PRINT_N time steps
DISPLAY_N = 5           # update robot configuration in viewer every DISPLAY_N time steps

######################### ROBOT PARAMS ########################

# traj file path
LIPM_FILE = "/huron_traj_lipm.npz"
TSID_FILE = "/huron_traj_tsid.npz"
DATA_FILE_LIPM = str(os.path.dirname(os.path.abspath(__file__))) + LIPM_FILE
DATA_FILE_TSID = str(os.path.dirname(os.path.abspath(__file__))) + TSID_FILE
traj_file = '/LIPMtraj.mat'
traj_file_path = str(os.path.dirname(os.path.abspath(__file__))) + traj_file

# URDF file path
filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + "/models/huron"
urdf = path + "/huron.urdf"

N_SIMULATION = 4000     # number of time steps simulated
dt = 0.002              # controller time step
T_pre = 1             # simulation time before starting to walk
T_post = 1           # simulation time after walking

nv = 18

lxp = 0.165         # foot length in positive x direction
lxn = 0.050         # foot length in negative x direction
lyp = 0.041         # foot length in positive y direction
lyn = 0.041         # foot length in negative y direction

lz = 0.095          # foot sole height with respect to ankle joint
mu = 0.3            # friction coefficient
fMin = 5.0          # minimum normal force
fMax = 1000.0       # maximum normal force

rf_frame_name = "r_ankle_roll_joint"        # right foot frame name
lf_frame_name = "l_ankle_roll_joint"        # left foot frame name
contactNormal = np.array([0.0, 0.0, 1.0])   # direction of the normal to the contact surface

# init contact point co-ordinates of the foot in ankle frame
contact_Point = np.ones((3, 4)) * (-lz)
contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
contact_Point[1, :] = [-lyn, lyp, lyn, lyp]
# for HURON the ankle frame is different
R = np.array([[0,0,-1],[0,1,0],[1,0,0]])
contact_Point = R @ contact_Point
contactNormal = R @ contactNormal

####################### TSID PARAMS ########################

w_com = 1.0             # weight of center of mass task
w_cop = 0.0             # weight of center of pressure task
w_am = 0.0              # weight of angular momentum task
w_foot = 1.0            # weight of the foot motion task
w_contact = 1e3         # weight of foot in contact (negative means infinite weight)
w_posture = 1e-2        # weight of joint posture task
w_forceRef = 0          # weight of force regularization task
w_torque_bounds = 0     # weight of the torque bounds
w_joint_bounds = 0      # weight of the joint bounds

tau_max_scaling = 1     # scaling factor of torque bounds
v_max_scaling = 0.8

kp_contact = 10.0       # proportional gain of contact constraint
kp_foot = 100.0         # proportional gain of foot task
kp_com = 25.0           # proportional gain of center of mass task
kp_am = 0.0             # proportional gain of angular momentum task
kp_posture = 10.0          # proportional gain of joint posture task
gain_vector = np.array([10, 10, 5, 1, 1, 1, 10, 10, 5, 1, 1, 1]) # kp_posture * np.ones(nv - 6)
masks_posture = np.ones(nv - 6)

################### LMPC PARAMS ###########################

alpha = 10 ** (2)  # CoP error squared cost weight
beta = 0  # CoM position error squared cost weight
gamma = 10 ** (-1)  # CoM velocity error squared cost weight
h = 0.598  # fixed CoM height
g = 9.81  # norm of the gravity vector

dt_mpc = 0.1  # sampling time interval
T_step = 0.8  # time needed for every step
step_length = 0.2 # fixed step length
step_width = 0.155 # dist between the feet
step_height = 0.05  # fixed step height

nb_steps = 6 # number of desired walking steps

foot_step_0 = np.array([0.0, -step_width/2])  # initial foot step position in x-y