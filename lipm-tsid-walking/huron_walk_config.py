# Configuration file for LMPC trajecotry generation
import os

import numpy as np
import pinocchio as pin

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

DATA_FILE_LIPM = str(os.path.dirname(os.path.abspath(__file__))) + "huron_traj_lipm.npz"
DATA_FILE_TSID = str(os.path.dirname(os.path.abspath(__file__))) + "huron_traj_tsid.npz"

# robot parameters
# ----------------------------------------------
# URDF file path
filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + "/../models/huron"
urdf = path + "/huron.urdf"
# srdf = path + "/srdf/romeo_collision.srdf"
nv = 18

lxp = 0.165  # foot length in positive x direction
lxn = 0.050  # foot length in negative x direction
lyp = 0.041  # foot length in positive y direction
lyn = 0.041  # foot length in negative y direction
lz = 0.095  # foot sole height with respect to ankle joint

mu = 0.3  # friction coefficient
fMin = 5.0  # minimum normal force
fMax = 1000.0  # maximum normal force
rf_frame_name = "r_ankle_roll_joint"  # right foot frame name
lf_frame_name = "l_ankle_roll_joint"  # left foot frame name
contactNormal = np.array([0.0, 0.0, 1.0])  # direction of the normal to the contact surface

# init contact point co-ordinates of the foot in ankle frame
contact_Point = np.ones((3, 4)) * (-lz)
contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
contact_Point[1, :] = [-lyn, lyp, lyn, lyp]
# Rotation matrix of HURON's Ankle frame wrt World frame
R = np.array([[0,0,-1],[0,1,0],[1,0,0]])
# Calc contact points and normal in Ankle frame
contact_Point = R @ contact_Point
contactNormal = R @ contactNormal

# configuration for LIPM trajectory optimization
# ----------------------------------------------
alpha = 10 ** (2)  # CoP error squared cost weight
beta = 0  # CoM position error squared cost weight
gamma = 10 ** (-1)  # CoM velocity error squared cost weight
h = 0.598  # fixed CoM height
g = 9.81  # norm of the gravity vector

dt_mpc = 0.1  # sampling time interval
T_step = 0.8  # time needed for every step
step_length = 0.2  # fixed step length
step_width = 0.155 # dist between the feet
step_height = 0.1  # fixed step height
nb_steps = 4  # number of desired walking steps

foot_step_0 = np.array([0.0, -step_width/2])  # initial foot step position in x-y

# configuration for TSID
# ----------------------------------------------
dt = 0.002  # controller time step
T_pre = 3  # simulation time before starting to walk
T_post = 1.5  # simulation time after walking

w_com = 1.0  # weight of center of mass task
w_cop = 0.0  # weight of center of pressure task
w_am = 1e-4  # weight of angular momentum task
w_foot = 1e0  # weight of the foot motion task
w_contact = 1e2  # weight of the foot in contact
w_posture = 1e-4  # weight of joint posture task
w_forceRef = 1e-5  # weight of force regularization task
w_torque_bounds = 0.0  # weight of the torque bounds
w_joint_bounds = 0.0

tau_max_scaling = 1.45  # scaling factor of torque bounds
v_max_scaling = 0.8

kp_contact = 10.0  # proportional gain of contact constraint
kp_foot = 10.0  # proportional gain of contact constraint
kp_com = 10.0  # proportional gain of center of mass task
kp_am = 10.0  # proportional gain of angular momentum task
kp_posture = 1.0  # proportional gain of joint posture task
gain_vector = kp_posture * np.ones(nv - 6)
masks_posture = np.ones(nv - 6)

# configuration for viewer
# ----------------------------------------------
viewer = pin.visualize.GepettoVisualizer
PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 20  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [
    3.578777551651001,
    1.2937744855880737,
    0.8885031342506409,
    0.4116811454296112,
    0.5468055009841919,
    0.6109083890914917,
    0.3978860676288605,
]
