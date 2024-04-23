import numpy as np
import matplotlib.pyplot as plt
from functions import *

######################## Walk parameters#########################

NSteps = 3              # number of steps
Ts = 1                  # swing phase time
dt = 0.01               # interval 

fl = 0.215              # foot length in X
fw = 0.082              # foot width in Y
x_off = 0.05            # foot center offset in X
y_off = fw / 2          # foot center offset in Y

step_len = 0.4          # step length in X
step_width = 0.155      # step width in Y
swing_height = 0.1

zRobot = 1.1227         # Actual robot height
zc = 1                  # Desired COM height

g = 9.81                # gravity

# step param matrix
sx, sy = step_params(step_len, step_width, NSteps)

######################## LIPM Walking ############################
# LIPM dynamics Params
Tc = np.sqrt(zc / g)
C = np.cosh(Ts / Tc)
S = np.sinh(Ts / Tc)

# Eval function Params
a = 10
b = 1
D = a * (C - 1) ** 2 + b * (S / Tc) ** 2

# Init Vars
px0, py0 = 0, 0             # LIPM initial foot hold
xi, yi = px0, py0           # Initial LIPM Pos
vxi, vyi = 0, 0             # Initial LIPM Vel

p_x, p_y = px0, py0         # LIPM foot hold
x, y = 0, 0                 # LIPM Pos
vx, vy = 0, 0               # LIPM Vel

# CoM Pos
x_com = []
y_com = []
z_com = []
# CoM Vel
vx_com = []
vy_com = []
vz_com = []

# z_com = np.append(z_com, zc * np.ones(len(time) - len(z_com)))
# vz_com = np.append(vz_com, np.zeros(len(time) - len(vz_com)))
n = -1
T = 0
fc_x, fc_y = [], []

for i in range(0,len(sx)):
    fc_x.append(p_x)
    fc_y.append(p_y)
    
    x_bar = sx[n+1] / 2
    y_bar = (-1) ** n * sy[n+1] / 2
    vx_bar = x_bar * (C + 1) / (Tc * S)
    vy_bar = y_bar * (C - 1) / (Tc * S)
    
    x_d = p_x + x_bar
    y_d = p_y + y_bar
    vx_d = vx_bar
    vy_d = vy_bar
    
    p_x = -(a * (C - 1) / D) * (x_d - C * xi - Tc * S * vxi) - (b * S / (Tc * D)) * (
            vx_d - S / Tc * xi - C * vxi)
    p_y = -(a * (C - 1) / D) * (y_d - C * yi - Tc * S * vyi) - (b * S / (Tc * D)) * (
            vy_d - S / Tc * yi - C * vyi)
    
    t = np.arange(T + dt, T + Ts + dt, dt)
    
    x = (xi - p_x) * np.cosh((t - T) / Tc) + Tc * vxi * np.sinh((t - T) / Tc) + p_x
    y = (yi - p_y) * np.cosh((t - T) / Tc) + Tc * vyi * np.sinh((t - T) / Tc) + p_y
    vx = (xi - p_x) / Tc * np.sinh((t - T) / Tc) + vxi * np.cosh((t - T) / Tc)
    vy = (yi - p_y) / Tc * np.sinh((t - T) / Tc) + vyi * np.cosh((t - T) / Tc)
    
    x_com = np.append(x_com, x)
    y_com = np.append(y_com, y)
    vx_com = np.append(vx_com, vx)
    vy_com = np.append(vy_com, vy)
    
    T = T + Ts
    n = n + 1
    p_x = p_x + sx[n]
    p_y = p_y - ((-1) ** (n)) * sy[n]
    xi = x[-1]
    yi = y[-1]
    vxi = vx[-1]
    vyi = vy[-1]


fc_x.append(p_x)
fc_y.append(p_y)
fcp_x = fc_x.copy()
fcp_y = fc_y.copy()
fcp_y[-1] = 0
fc_y[0] = -0.155 / 2

# Calculate foot centers and bottom right corner pts for plotting
xLeft = [fc_x[0] - x_off]
yLeft = [fc_y[0] + 2*y_off]
for i in range(len(sx)):
    xLeft.append(fc_x[i + 1] - x_off)
    yLeft.append(fc_y[i + 1] - y_off)

plt.figure()
plt.grid(True)
plt.axis('equal')

# Plot footsteps
for i in range(len(xLeft)):
    plt.gca().add_patch(plt.Rectangle((xLeft[i], yLeft[i]), fl, fw, edgecolor='b', linewidth=2))


# planning time period
time_period = (NSteps + 2)*Ts
time = np.arange(0, time_period, dt)

# Plot CoM trajectory
plt.plot(x_com, y_com, 'r', linewidth=2.5)
plt.plot(fc_x, fc_y, 'ob')
plt.plot(fcp_x, fcp_y, '--o', markersize=7.5, markerfacecolor='r')

plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend(['CoM Traj', 'LIPM ZMP Traj', 'Foot Center'])

# Position and velocity plots
plt.figure()

plt.subplot(2, 2, 1)
plt.plot(time, x_com)
plt.xlabel('time (sec)')
plt.ylabel('x (m)')
plt.title('X_{com} vs t')

plt.subplot(2, 2, 2)
plt.plot(time, y_com)
plt.xlabel('time (sec)')
plt.ylabel('y (m)')
plt.title('Y_{com} vs t')

plt.subplot(2, 2, 3)
plt.plot(time, vx_com)
plt.xlabel('time (sec)')
plt.ylabel('Vx (m/s)')
plt.title('V_x_{com} vs t')

plt.subplot(2, 2, 4)
plt.plot(time, vy_com)
plt.xlabel('time (sec)')
plt.ylabel('Vy (m/s)')
plt.title('V_y_{com} vs t')

plt.tight_layout()
plt.show()

################# Foot Trajectories ###############################


# timesteps = np.arange(1, time_period + 1, Ts)
# foot_right_wp = np.array([fc_x[0], fc_x[0], fc_x[2], fc_x[2], fc_x[4], fc_x[4],
#                           fc_y[0], fc_y[0], fc_y[2], fc_y[2], fc_y[4], fc_y[4],
#                           np.zeros(6)])
# foot_left_wp = np.array([fc_x[1], fc_x[1], fc_x[1], fc_x[3], fc_x[3], fc_x[5],
#                          fc_y[1], fc_y[1], fc_y[1], fc_y[3], fc_y[3], fc_y[5],
#                          np.zeros(6)])

# footPos_right = np.ones((3, len(np.arange(0, 1 + dt, dt)))) * foot_right_wp[:, 0:1]
# footPos_left = np.ones((3, len(np.arange(0, 1 + dt, dt)))) * foot_left_wp[:, 0:1]
# footVel_right = np.zeros((3, len(np.arange(0, 1 + dt, dt))))
# footVel_left = np.zeros((3, len(np.arange(0, 1 + dt, dt))))

# for i in range(N):
#     footpos_right_i = foot_right_wp[:, i]
#     footpos_right_f = foot_right_wp[:, i + 1]
#     footpos_left_i = foot_left_wp[:, i]
#     footpos_left_f = foot_left_wp[:, i + 1]
#     ti = timesteps[i]
#     tf = timesteps[i + 1]
#     swing_r = swing_height if i % 2 == 0 else 0
#     swing_l = 0 if i % 2 == 0 else swing_height
#     q_r, qd_r, qdd_r = getSwingFootTraj(footpos_right_i, footpos_right_f, swing_r, ti + dt, tf, dt)
#     q_l, qd_l, qdd_l = getSwingFootTraj(footpos_left_i, footpos_left_f, swing_l, ti + dt, tf, dt)
#     footPos_right = np.concatenate((footPos_right, q_r), axis=1)
#     footPos_left = np.concatenate((footPos_left, q_l), axis=1)
#     footVel_right = np.concatenate((footVel_right, qd_r), axis=1)
#     footVel_left = np.concatenate((footVel_left, qd_l), axis=1)

# # desiredStates = convertMat2gazebo(np.array([x_com, y_com, z_com]), np.array([vx_com, vy_com, vz_com]),
#                                 #   footPos_right, footVel_right, footPos_left, footVel_left)
# # desJointStates = getJointStates(desiredStates)

# px, py = px0, py0
# n = 0
# fig = plt.figure()
# for i in range(len(time)):
#     if i % 100 == 0:
#         px = fcp_x[n]
#         py = fcp_y[n]
#         n += 1
#     plt.plot([px, x_com[i]], [py, y_com[i]], '-b', linewidth=2)
#     plt.plot(x_com[i], y_com[i], 'ro', markersize=15)
#     plt.plot(x_com[:i + 1], y_com[:i + 1])
#     plt.plot(footPos_right[0, :i + 1], footPos_right[1, :i + 1])
#     plt.plot(footPos_left[0, :i + 1], footPos_left[1, :i + 1])
#     plt.axis([-0.4, 1.4, -0.4, 0.4])
#     plt.grid(True)
#     plt.pause(dt)
#     plt.clf()

