import time
import numpy as np
from numpy import nan
import plot_utils as plut
import huron_config as conf
import matplotlib.pyplot as plt
from numpy.linalg import norm as norm
from tsid_biped_huron import tsid_biped

################# Init TSID ###################

huron = tsid_biped(conf)

############ Load Trajectory Data #############

try:
    data = np.load(conf.DATA_FILE_TSID)
except FileNotFoundError as e:
    print("Trajectory data file not found:  " + conf.DATA_FILE_TSID)
    raise e

contact_phase = data["contact_phase"]

com_pos_ref = np.asarray(data["com"])
com_vel_ref = np.asarray(data["dcom"])
com_acc_ref = np.asarray(data["ddcom"])

RF_pos_ref = np.asarray(data["x_RF"])
RF_vel_ref = np.asarray(data["dx_RF"])
RF_acc_ref = np.asarray(data["ddx_RF"])

LF_pos_ref = np.asarray(data["x_LF"])
LF_vel_ref = np.asarray(data["dx_LF"])
LF_acc_ref = np.asarray(data["ddx_LF"])

cop_ref = np.asarray(data["cop"])

# offset foot traj to match ankle joint
p_rf = huron.get_placement_RF().translation
offset = (p_rf - RF_pos_ref[:, 0]).reshape((3,1))
RF_pos_ref += offset
LF_pos_ref += offset
 
RF_pos_ref[:,-1] = RF_pos_ref[:,-2] # bug fixz

################### INIT VARS ###################
# Init vars to store sim data
# No of data points
N = data["com"].shape[1]
N_pre = int(conf.T_pre / conf.dt) # pre traj sim time
N_post = int(conf.T_post / conf.dt) # post traj sim time
# com data
com_pos = np.empty((3, N + N_post)) * nan
com_vel = np.empty((3, N + N_post)) * nan
com_acc = np.empty((3, N + N_post)) * nan
com_acc_des = (np.empty((3, N + N_post)) * nan)  # acc_des = acc_ref - Kp*pos_err - Kd*vel_err
# left foot data
x_LF = np.empty((3, N + N_post)) * nan
dx_LF = np.empty((3, N + N_post)) * nan
ddx_LF = np.empty((3, N + N_post)) * nan
ddx_LF_des = np.empty((3, N + N_post)) * nan
# right foot data
x_RF = np.empty((3, N + N_post)) * nan
dx_RF = np.empty((3, N + N_post)) * nan
ddx_RF = np.empty((3, N + N_post)) * nan
ddx_RF_des = np.empty((3, N + N_post)) * nan
# contact force data
f_RF = np.zeros((6, N + N_post))
f_LF = np.zeros((6, N + N_post))
# cop data
cop_RF = np.zeros((2, N + N_post))
cop_LF = np.zeros((2, N + N_post))
# torque data
tau = np.zeros((huron.robot.na, N + N_post))
# joint angles and vel
q_log = np.zeros((huron.robot.nq, N + N_post))
v_log = np.zeros((huron.robot.nv, N + N_post))

########### CONTROL LOOP & SIMULATION ############

t = -conf.T_pre # sim start time
q, v = huron.q, huron.v
q_sim = [] # store joint angles

input("Press enter to start")

for i in range(-N_pre, N + N_post):
    time_start = time.time()

    # switch foot contacts
    if i == 0:
        print("Starting to walk (remove contact left foot)")
        huron.remove_contact_LF()
    elif i > 0 and i < N - 1:
        if contact_phase[i] != contact_phase[i - 1]:
            print("Time %.3f Changing contact phase from %s to %s"
                % (t, contact_phase[i - 1], contact_phase[i]))
            if contact_phase[i] == "left":
                huron.add_contact_LF()
                huron.remove_contact_RF()
            else:
                huron.add_contact_RF()
                huron.remove_contact_LF()

    # set com, LF and RF reference
    if i < 0:
        huron.set_com_ref(com_pos_ref[:, 0], 0 * com_vel_ref[:, 0], 0 * com_acc_ref[:, 0])
    elif i < N:
        huron.set_com_ref(com_pos_ref[:, i], com_vel_ref[:, i], com_acc_ref[:, i])
        huron.set_LF_3d_ref(LF_pos_ref[:, i], LF_vel_ref[:, i], LF_acc_ref[:, i])
        huron.set_RF_3d_ref(RF_pos_ref[:, i], RF_vel_ref[:, i], RF_acc_ref[:, i])

    # solve HQP
    HQPData = huron.formulation.computeProblemData(t, q, v)
    sol = huron.solver.solve(HQPData)
    dv = huron.formulation.getAccelerations(sol)
    if sol.status != 0:
        print("QP problem could not be solved! Error code:", sol.status)
        break
    if norm(v, 2) > 40.0:
        print("Time %.3f Velocities are too high, stop everything!" % (t), norm(v))
        break

    # store sim data
    if i > 0:
        q_log[:, i] = q
        v_log[:, i] = v
        tau[:, i] = huron.formulation.getActuatorForces(sol)
    if i >= 0:
        com_pos[:, i] = huron.robot.com(huron.formulation.data())
        com_vel[:, i] = huron.robot.com_vel(huron.formulation.data())
        com_acc[:, i] = huron.comTask.getAcceleration(dv)
        com_acc_des[:, i] = huron.comTask.getDesiredAcceleration

        x_LF[:, i], dx_LF[:, i], ddx_LF[:, i] = huron.get_LF_3d_pos_vel_acc(dv)
        if not huron.contact_LF_active:
            ddx_LF_des[:, i] = huron.leftFootTask.getDesiredAcceleration[:3]
        
        x_RF[:, i], dx_RF[:, i], ddx_RF[:, i] = huron.get_RF_3d_pos_vel_acc(dv)
        if not huron.contact_RF_active:
            ddx_RF_des[:, i] = huron.rightFootTask.getDesiredAcceleration[:3]

        if huron.formulation.checkContact(huron.contactRF.name, sol):
            T_RF = huron.contactRF.getForceGeneratorMatrix
            f_RF[:, i] = T_RF.dot(huron.formulation.getContactForce(huron.contactRF.name, sol))
            if f_RF[2, i] > 1e-3:
                cop_RF[0, i] = f_RF[4, i] / f_RF[2, i]
                cop_RF[1, i] = -f_RF[3, i] / f_RF[2, i]
        if huron.formulation.checkContact(huron.contactLF.name, sol):
            T_LF = huron.contactRF.getForceGeneratorMatrix
            f_LF[:, i] = T_LF.dot(huron.formulation.getContactForce(huron.contactLF.name, sol))
            if f_LF[2, i] > 1e-3:
                cop_LF[0, i] = f_LF[4, i] / f_LF[2, i]
                cop_LF[1, i] = -f_LF[3, i] / f_LF[2, i]

    # print stuff to terminal
    if i % conf.PRINT_N == 0:
        print("Time %.3f" % (t))
        print("\tnormal force %s: %.1f" % (huron.contactRF.name.ljust(20, "."), f_RF[2, i]))
        print("\tnormal force %s: %.1f" % (huron.contactLF.name.ljust(20, "."), f_LF[2, i]))
        # com tracking error
        print("\ttracking err %s: %.3f" % (huron.comTask.name.ljust(20, "."), np.linalg.norm(huron.comTask.position_error, 2)))
        # lf tracking error
        print("\ttracking err %s: %.3f" % (huron.leftFootTask.name.ljust(20, "."), np.linalg.norm(huron.leftFootTask.position_error, 2)))
        # rf tracking error
        print("\ttracking err %s: %.3f" % (huron.rightFootTask.name.ljust(20, "."), np.linalg.norm(huron.rightFootTask.position_error, 2)))

    q, v = huron.integrate_dv(q, v, dv, conf.dt)
    t += conf.dt

    q_sim.append(q)

    if i % conf.DISPLAY_N == 0:
        huron.viz.display(q)

    time_spent = time.time() - time_start
    if time_spent < conf.dt:
        time.sleep(conf.dt - time_spent)

# Replay sim
while True:
    replay = input("Play video again? [Y]/n: ") or "Y"
    if replay.lower() == "y":
        for q in q_sim:
            huron.viz.display(q)
    else:
        break

#################### PLOT STUFF ###################
    
time = np.arange(0.0, (N + N_post) * conf.dt, conf.dt)

PLOT_COM = 1
PLOT_COP = 0
PLOT_FOOT_TRAJ = 1
PLOT_TORQUES = 0
PLOT_JOINT_VEL = 0

if PLOT_COM:
    (f, ax) = plut.create_empty_figure(3, 1)
    for i in range(3):
        ax[i].plot(time, com_pos[i, :], label="CoM " + str(i))
        ax[i].plot(time[:N], com_pos_ref[i, :], "r:", label="CoM Ref " + str(i))
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoM [m]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

    (f, ax) = plut.create_empty_figure(3, 1)
    for i in range(3):
        ax[i].plot(time, com_vel[i, :], label="CoM Vel " + str(i))
        ax[i].plot(time[:N], com_vel_ref[i, :], "r:", label="CoM Vel Ref " + str(i))
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoM Vel [m/s]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

    (f, ax) = plut.create_empty_figure(3, 1)
    for i in range(3):
        ax[i].plot(time, com_acc[i, :], label="CoM Acc " + str(i))
        ax[i].plot(time[:N], com_acc_ref[i, :], "r:", label="CoM Acc Ref " + str(i))
        ax[i].plot(time, com_acc_des[i, :], "g--", label="CoM Acc Des " + str(i))
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoM Acc [m/s^2]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if PLOT_COP:
    (f, ax) = plut.create_empty_figure(2, 1)
    for i in range(2):
        ax[i].plot(time, cop_LF[i, :], label="CoP LF " + str(i))
        ax[i].plot(time, cop_RF[i, :], label="CoP RF " + str(i))
        # ax[i].plot(time[:N], cop_ref[i,:], label='CoP ref '+str(i))
        if i == 0:
            ax[i].plot(
                [time[0], time[-1]],
                [-conf.lxn, -conf.lxn],
                ":",
                label="CoP Lim " + str(i),
            )
            ax[i].plot(
                [time[0], time[-1]],
                [conf.lxp, conf.lxp],
                ":",
                label="CoP Lim " + str(i),
            )
        elif i == 1:
            ax[i].plot(
                [time[0], time[-1]],
                [-conf.lyn, -conf.lyn],
                ":",
                label="CoP Lim " + str(i),
            )
            ax[i].plot(
                [time[0], time[-1]],
                [conf.lyp, conf.lyp],
                ":",
                label="CoP Lim " + str(i),
            )
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoP [m]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if PLOT_FOOT_TRAJ:
    for i in range(3):
        plt.figure()
        plt.plot(time, x_RF[i, :], label="x RF " + str(i))
        plt.plot(time[:N], RF_pos_ref[i, :], ":", label="x RF ref " + str(i))
        plt.plot(time, x_LF[i, :], label="x LF " + str(i))
        plt.plot(time[:N], LF_pos_ref[i, :], ":", label="x LF ref " + str(i))
        plt.legend()

    # for i in range(3):
    #    plt.figure()
    #    plt.plot(time, dx_RF[i,:], label='dx RF '+str(i))
    #    plt.plot(time[:N], RF_vel_ref[i,:], ':', label='dx RF ref '+str(i))
    #    plt.plot(time, dx_LF[i,:], label='dx LF '+str(i))
    #    plt.plot(time[:N], LF_vel_ref[i,:], ':', label='dx LF ref '+str(i))
    #    plt.legend()
    #
    # for i in range(3):
    #    plt.figure()
    #    plt.plot(time, ddx_RF[i,:], label='ddx RF '+str(i))
    #    plt.plot(time[:N], RF_acc_ref[i,:], ':', label='ddx RF ref '+str(i))
    #    plt.plot(time, ddx_RF_des[i,:], '--', label='ddx RF des '+str(i))
    #    plt.plot(time, ddx_LF[i,:], label='ddx LF '+str(i))
    #    plt.plot(time[:N], LF_acc_ref[i,:], ':', label='ddx LF ref '+str(i))
    #    plt.plot(time, ddx_LF_des[i,:], '--', label='ddx LF des '+str(i))
    #    plt.legend()

if PLOT_TORQUES:
    plt.figure()
    for i in range(huron.robot.na):
        tau_normalized = (
            2
            * (tau[i, :] - huron.tau_min[i])
            / (huron.tau_max[i] - huron.tau_min[i])
            - 1
        )
        # plot torques only for joints that reached 50% of max torque
        if np.max(np.abs(tau_normalized)) > 0.5:
            plt.plot(
                time, tau_normalized, alpha=0.5, label=huron.model.names[i + 2]
            )
    plt.plot([time[0], time[-1]], 2 * [-1.0], ":")
    plt.plot([time[0], time[-1]], 2 * [1.0], ":")
    plt.gca().set_xlabel("Time [s]")
    plt.gca().set_ylabel("Normalized Torque")
    leg = plt.legend()
    leg.get_frame().set_alpha(0.5)

if PLOT_JOINT_VEL:
    plt.figure()
    for i in range(huron.robot.na):
        v_normalized = (
            2
            * (v_log[6 + i, :] - huron.v_min[i])
            / (huron.v_max[i] - huron.v_min[i])
            - 1
        )
        # plot v only for joints that reached 50% of max v
        if np.max(np.abs(v_normalized)) > 0.5:
            plt.plot(time, v_normalized, alpha=0.5, label=huron.model.names[i + 2])
    plt.plot([time[0], time[-1]], 2 * [-1.0], ":")
    plt.plot([time[0], time[-1]], 2 * [1.0], ":")
    plt.gca().set_xlabel("Time [s]")
    plt.gca().set_ylabel("Normalized Joint Vel")
    leg = plt.legend()
#    leg.get_frame().set_alpha(0.5)

plt.show()
