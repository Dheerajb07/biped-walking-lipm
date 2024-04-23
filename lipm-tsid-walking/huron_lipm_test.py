# test TSID setup for HURON
import time
import tsid

import numpy as np
import pinocchio as pin
import plot_utils as plut
from scipy.io import loadmat
import matplotlib.pyplot as plt

from tsid_biped_huron import tsid_biped
import huron_config as conf

huron = tsid_biped(conf)

###################### Load Trajectory data ######################

# load trajectory from mat file
traj_data = loadmat(conf.traj_file_path,simplify_cells=True)
traj_data = traj_data["LIPMtraj"]

com_pos_ref = np.asarray(traj_data["com_pos"])
com_vel_ref = np.asarray(traj_data["com_vel"])
com_acc_ref = np.asarray(traj_data["com_acc"])

rf_pos_ref = np.asarray(traj_data["rf_pos"]) + np.array([0,0,conf.lz]).reshape((3,1))
rf_vel_ref = np.asarray(traj_data["rf_vel"])
rf_acc_ref = np.asarray(traj_data["rf_acc"])

lf_pos_ref = np.asarray(traj_data["lf_pos"]) + np.array([0,0,conf.lz]).reshape((3,1))
lf_vel_ref = np.asarray(traj_data["lf_vel"])
lf_acc_ref = np.asarray(traj_data["lf_acc"])

# data = np.load('huron_traj_tsid.npz')

# contact_phase = data["contact_phase"]
# cop_ref = np.asarray(data["cop"])

# com_pos_ref = np.asarray(data["com"])
# com_vel_ref = np.asarray(data["dcom"])
# com_acc_ref = np.asarray(data["ddcom"])

# rf_pos_ref = np.asarray(data["x_RF"]) + np.array([0,0,conf.lz]).reshape((3,1))
# rf_vel_ref = np.asarray(data["dx_RF"])
# rf_acc_ref = np.asarray(data["ddx_RF"])

# lf_pos_ref = np.asarray(data["x_LF"]) + np.array([0,0,conf.lz]).reshape((3,1))
# lf_vel_ref = np.asarray(data["dx_LF"])
# lf_acc_ref = np.asarray(data["ddx_LF"])

################### CONTROL LOOP & SIMULATION ####################
print("Simulating ...")

N = com_pos_ref.shape[1]       # no. of simulation steps
N_pre = int(conf.T_pre / conf.dt)
N_post = int(conf.T_post / conf.dt)

# Initialize arrays with NaN values
nan_value = np.nan
com_pos = np.full((3, N + N_post), nan_value)
com_vel = np.full((3, N + N_post), nan_value)
com_acc = np.full((3, N + N_post), nan_value)
lf_pos = np.full((3, N + N_post), nan_value)
lf_vel = np.full((3, N + N_post), nan_value)
lf_acc = np.full((3, N + N_post), nan_value)
rf_pos = np.full((3, N + N_post), nan_value)
rf_vel = np.full((3, N + N_post), nan_value)
rf_acc = np.full((3, N + N_post), nan_value)

# sampleCom = huron.trajCom.computeNext()
# samplePosture = huron.trajPosture.computeNext()

t = 0.0
q = huron.q
v = huron.v
phase = 0
contact_phase = "both"
for i in range(-N_pre,N + N_post):
    time_start = time.time()
    # add/break contacts 
    if i>=0 and i<N:
        if i%(conf.T_step/conf.dt) == 0:
            if i!=0 and phase%2 == 0:
                contact_phase = "left"
                huron.add_contact_RF()
                huron.remove_contact_LF()
            elif i!=0 and phase%2 != 0:
                contact_phase = "right"
                if phase != 1:
                    huron.add_contact_LF()
                huron.remove_contact_RF()
            print(phase,contact_phase)       
            phase = phase + 1         

    # set ref traj
    if i < 0:
        huron.set_com_ref(huron.sample_com.value(), huron.sample_com.derivative(), huron.sample_com.second_derivative())
        # huron.set_LF_3d_ref(huron.sampleLF.value()[:3], huron.sampleLF.derivative()[:3], huron.sampleLF.second_derivative()[:3])
        # huron.set_RF_3d_ref(huron.sampleRF.value()[:3], huron.sampleRF.derivative()[:3], huron.sampleRF.second_derivative()[:3])
    elif i < N:
        huron.set_com_ref(com_pos_ref[:,i],com_vel_ref[:,i],com_acc_ref[:,i])
        huron.set_LF_3d_ref(lf_pos_ref[:, i], lf_vel_ref[:, i], lf_acc_ref[:, i])
        huron.set_RF_3d_ref(rf_pos_ref[:, i], rf_pos_ref[:, i], rf_acc_ref[:, i])

    HQPData = huron.formulation.computeProblemData(t, q, v)
    if i == 0: HQPData.print_all()

    sol = huron.solver.solve(HQPData)
    if sol.status != 0:
        print("QP problem could not be solved! Error code:", sol.status)
        print("Iteration: ",i)
        break
    if np.linalg.norm(v, 2) > 40.0:
        print("Time %.3f Velocities are too high, stop everything!" % (t), np.linalg.norm(v))
        break

    tau = huron.formulation.getActuatorForces(sol)
    dv = huron.formulation.getAccelerations(sol)

    if i >= 0:
        com_pos[:, i] = huron.robot.com(huron.formulation.data())
        com_vel[:, i] = huron.robot.com_vel(huron.formulation.data())
        com_acc[:, i] = huron.comTask.getAcceleration(dv)
    
        rf_pos[:,i], rf_vel[:,i], rf_acc[:,i] = huron.get_RF_3d_pos_vel_acc(dv)

        lf_pos[:,i], lf_vel[:,i], lf_acc[:,i] = huron.get_LF_3d_pos_vel_acc(dv)

    if i % conf.PRINT_N == 0:
        print("Time %.3f" % (t))
        if huron.formulation.checkContact(huron.contactRF.name, sol):
            f = huron.formulation.getContactForce(huron.contactRF.name, sol)
            print(
                "\tnormal force %s: %.1f"
                % (huron.contactRF.name.ljust(20, "."), huron.contactRF.getNormalForce(f))
            )

        if huron.formulation.checkContact(huron.contactLF.name, sol):
            f = huron.formulation.getContactForce(huron.contactLF.name, sol)
            print(
                "\tnormal force %s: %.1f"
                % (huron.contactLF.name.ljust(20, "."), huron.contactLF.getNormalForce(f))
            )
        # com tracking error
        print("\ttracking err %s: %.3f" % (huron.comTask.name.ljust(20, "."), np.linalg.norm(huron.comTask.position_error, 2)))
        # lf tracking error
        print("\ttracking err %s: %.3f" % (huron.leftFootTask.name.ljust(20, "."), np.linalg.norm(huron.leftFootTask.position_error, 2)))
        # rf tracking error
        print("\ttracking err %s: %.3f" % (huron.rightFootTask.name.ljust(20, "."), np.linalg.norm(huron.rightFootTask.position_error, 2)))

        print("\t||v||: %.3f\t ||dv||: %.3f" % (np.linalg.norm(v, 2), np.linalg.norm(dv)))
        # print("Joint Acc: ",dv)
        # print("Joint Torque: ",tau)

    q, v = huron.integrate_dv(q, v, dv, conf.dt)
    t += conf.dt

    if i % conf.DISPLAY_N == 0:
        huron.viz.display(q)

    time_spent = time.time() - time_start
    if time_spent < conf.dt:
        time.sleep(conf.dt - time_spent)

############## PLOTS ##################
time = np.arange(0.0, (N + N_post) * conf.dt, conf.dt)

# Plot COM
# Create empty figure and axes for positions
f, ax = plt.subplots(3, 1)

labels_positions = ['x', 'y', 'z']
for i, label in enumerate(labels_positions):
    ax[i].plot(time, com_pos[i, :], label=label)
    ax[i].plot(time[:N], com_pos_ref[i, :], "r:", label=label + " Ref")
    ax[i].set_xlabel(r"Time (s)")
    ax[i].set_ylabel(r"$\mathrm{" + label + r"}$ (m)")
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()

# Create empty figure and axes for velocities
f, ax = plt.subplots(3, 1)

labels_velocities = ['xdot', 'ydot', 'zdot']
for i, label in enumerate(labels_velocities):
    ax[i].plot(time, com_vel[i, :], label=label)
    ax[i].plot(time[:N], com_vel_ref[i, :], "r:", label=label + " Ref")
    ax[i].set_xlabel(r"Time (s)")
    ax[i].set_ylabel(r"$\dot{" + label + r"}$ (m/s)")
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()

# Create empty figure and axes for accelerations
f, ax = plt.subplots(3, 1)

labels_accelerations = ['xddot', 'yddot', 'zddot']
for i, label in enumerate(labels_accelerations):
    ax[i].plot(time, com_acc[i, :], label=label)
    ax[i].plot(time[:N], com_acc_ref[i, :], "r:", label=label + " Ref")
    # ax[i].plot(time, com_acc_des[i, :], "g--", label="CoM Acc Des " + str(i))
    ax[i].set_xlabel(r"Time (s)")
    ax[i].set_ylabel(r"$\ddot{" + label + r"}$ (m/s$^2$)")
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()

# Plot LF

# Plot RF