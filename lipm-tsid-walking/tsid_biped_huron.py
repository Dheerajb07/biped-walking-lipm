import time
import numpy as np
import pinocchio as pin
import tsid

class tsid_biped:
    """ 
    TSID formulation for a biped robot
    - Center of mass task
    - Postural task
    - 6d rigid contact constraint for both feet
    - Regularization task for contact forces
    """
    def __init__(self,conf):
        self.conf = conf
        ###################### TSID SETUP ##########################
        # load robot model
        robot = tsid.RobotWrapper(conf.urdf,[conf.path],pin.JointModelFreeFlyer(), False)
        model = robot.model()
        # intial state vectors
        q = np.array(
            [
                0,              
                0,              
                1.0627,         
                0,              
                0,
                0,
                1,
                0.0000,
                0.0000,
                -0.3207,
                0.7572,
                -0.4365,
                0.0000,
                0.0000,
                0.0000,
                -0.3207,
                0.7572,
                -0.4365,
                0.0000,
            ]
        )
        v = np.zeros(robot.nv)

        # check if right and left foot frames mentioned in conf exist in model
        assert model.existFrame(conf.rf_frame_name)
        assert model.existFrame(conf.lf_frame_name)

        # formulate inverse dynamics
        formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()

        # init contact point co-ordinates of the foot in ankle frame
        contact_Point = conf.contact_Point

        # Add right foot (RF) contact constraint
        contactRF = tsid.Contact6d(
            "contact_rfoot",
            robot,
            conf.rf_frame_name,
            contact_Point,
            conf.contactNormal,
            conf.mu,
            conf.fMin,
            conf.fMax,
        )
        contactRF.setKp(conf.kp_contact * np.ones(6))
        contactRF.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        # get RF frame
        RF = model.getFrameId(conf.rf_frame_name)
        H_rf_ref = robot.framePosition(data, RF)
        # modify initial robot configuration so that foot is on the ground (z=0)
        q[2] -= H_rf_ref.translation[2] - conf.lz
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()
        H_rf_ref = robot.framePosition(data, RF)
        # set RF frame pos as reference and add rigid contact
        contactRF.setReference(H_rf_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactRF, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactRF, conf.w_forceRef)

        # Add left foot (LF) contact constraint
        contactLF = tsid.Contact6d(
            "contact_lfoot",
            robot,
            conf.lf_frame_name,
            contact_Point,
            conf.contactNormal,
            conf.mu,
            conf.fMin,
            conf.fMax,
        )
        contactLF.setKp(conf.kp_contact * np.ones(6))
        contactLF.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        # get LF frame
        LF = robot.model().getFrameId(conf.lf_frame_name)
        H_lf_ref = robot.framePosition(data, LF)
        # set LF frame pos as reference and add rigid contact
        contactLF.setReference(H_lf_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactLF, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactLF, conf.w_forceRef)

        # Add COM Task
        comTask = tsid.TaskComEquality("task-com", robot)
        comTask.setKp(conf.kp_com * np.ones(3))
        comTask.setKd(2.0 * np.sqrt(conf.kp_com) * np.ones(3))
        formulation.addMotionTask(comTask, conf.w_com, 1, 0.0)

        # Add Posture Task
        postureTask = tsid.TaskJointPosture("task-posture", robot)
        postureTask.setKp(conf.kp_posture * conf.gain_vector)
        postureTask.setKd(2.0 * np.sqrt(conf.kp_posture * conf.gain_vector))
        postureTask.setMask(conf.masks_posture)
        formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)

        # add Left-foot task
        leftFootTask = tsid.TaskSE3Equality("task-left-foot", robot, conf.lf_frame_name)
        leftFootTask.setKp(conf.kp_foot * np.ones(6))
        leftFootTask.setKd(2.0 * np.sqrt(conf.kp_foot) * np.ones(6))
        formulation.addMotionTask(leftFootTask, conf.w_foot, 1, 0.0)

        # add Right-foot task
        rightFootTask = tsid.TaskSE3Equality("task-right-foot", robot, conf.rf_frame_name)
        rightFootTask.setKp(conf.kp_foot * np.ones(6))
        rightFootTask.setKd(2.0 * np.sqrt(conf.kp_foot) * np.ones(6))
        formulation.addMotionTask(rightFootTask, conf.w_foot, 1, 0.0)
        
        # add torque limit constraint
        tau_max = conf.tau_max_scaling * robot.model().effortLimit[-robot.na :]
        tau_min = -tau_max
        actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", robot)
        actuationBoundsTask.setBounds(tau_min, tau_max)
        if conf.w_torque_bounds > 0.0:
            formulation.addActuationTask(actuationBoundsTask, conf.w_torque_bounds, 0, 0.0)

        # add joint-velocity limits constraint
        v_max = conf.v_max_scaling * robot.model().velocityLimit[-robot.na :]
        v_min = -v_max
        jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot, conf.dt)
        jointBoundsTask.setVelocityBounds(v_min, v_max)
        if conf.w_joint_bounds > 0.0:
            formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

        # Init HQP Solver
        self.solver = tsid.SolverHQuadProgFast("qp solver")
        self.solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

        # store variables
        self.robot = robot
        self.q = q
        self.v = v
        self.formulation = formulation
        self.contactRF = contactRF
        self.contactLF = contactLF
        self.comTask = comTask
        self.postureTask = postureTask
        self.rightFootTask = rightFootTask 
        self.RF = RF
        self.leftFootTask = leftFootTask
        self.LF = LF
        self.actuationBoundsTask = actuationBoundsTask
        self.jointBoundsTask = jointBoundsTask
        # contact phase
        self.contact_LF_active = True
        self.contact_RF_active = True

        # Init reference COM Trajectory
        com_ref = robot.com(data)
        self.trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
        self.sample_com = self.trajCom.computeNext()

        # Init reference joint trajectories
        q_ref = q[7:]
        self.trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)
        self.postureTask.setReference(self.trajPosture.computeNext())

        # Init reference foot trajectories
        self.trajLF = tsid.TrajectorySE3Constant("traj-left-foot", H_lf_ref)
        self.sampleLF = self.trajLF.computeNext()
        self.sample_LF_pos = self.sampleLF.value()
        self.sample_LF_vel = self.sampleLF.derivative()
        self.sample_LF_acc = self.sampleLF.second_derivative()

        self.trajRF = tsid.TrajectorySE3Constant("traj-right-foot", H_rf_ref)
        self.sampleRF = self.trajRF.computeNext()
        self.sample_RF_pos = self.sampleRF.value()
        self.sample_RF_vel = self.sampleRF.derivative()
        self.sample_RF_acc = self.sampleRF.second_derivative()

        ###################### Init Robot Visualizer #####################

        robot_display = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path], pin.JointModelFreeFlyer())
        viewer = pin.visualize.MeshcatVisualizer
        self.viz = viewer(
            robot_display.model,
            robot_display.collision_model,
            robot_display.visual_model,
        )

        print("Starting Meshcat ...")
        self.viz.initViewer(loadModel=True, open=True)
        self.viz.display(q)
        time.sleep(5)   # wait for 5 secs until meshcat starts

    def integrate_dv(self, q, v, dv, dt):
        v_mean = v + 0.5 * dt * dv
        v += dt * dv
        q = pin.integrate(self.robot.model(), q, dt * v_mean)
        return q, v

    def get_placement_LF(self):
        return self.robot.framePosition(self.formulation.data(), self.LF)

    def get_placement_RF(self):
        return self.robot.framePosition(self.formulation.data(), self.RF)

    def set_com_ref(self, pos, vel, acc):
        self.sample_com.value(pos)
        self.sample_com.derivative(vel)
        self.sample_com.second_derivative(acc)
        self.comTask.setReference(self.sample_com)

    def set_RF_3d_ref(self, pos, vel, acc):
        self.sample_RF_pos[:3] = pos
        self.sample_RF_vel[:3] = vel
        self.sample_RF_acc[:3] = acc
        self.sampleRF.value(self.sample_RF_pos)
        self.sampleRF.derivative(self.sample_RF_vel)
        self.sampleRF.second_derivative(self.sample_RF_acc)
        self.rightFootTask.setReference(self.sampleRF)

    def set_LF_3d_ref(self, pos, vel, acc):
        self.sample_LF_pos[:3] = pos
        self.sample_LF_vel[:3] = vel
        self.sample_LF_acc[:3] = acc
        self.sampleLF.value(self.sample_LF_pos)
        self.sampleLF.derivative(self.sample_LF_vel)
        self.sampleLF.second_derivative(self.sample_LF_acc)
        self.leftFootTask.setReference(self.sampleLF)

    def remove_contact_RF(self, transition_time=0.0):
        H_rf_ref = self.robot.framePosition(self.formulation.data(), self.RF)
        self.trajRF.setReference(H_rf_ref)
        self.rightFootTask.setReference(self.trajRF.computeNext())

        self.formulation.removeRigidContact(self.contactRF.name, transition_time)
        self.contact_RF_active = False

    def remove_contact_LF(self, transition_time=0.0):
        H_lf_ref = self.robot.framePosition(self.formulation.data(), self.LF)
        self.trajLF.setReference(H_lf_ref)
        self.leftFootTask.setReference(self.trajLF.computeNext())

        self.formulation.removeRigidContact(self.contactLF.name, transition_time)
        self.contact_LF_active = False

    def add_contact_RF(self, transition_time=0.0):
        H_rf_ref = self.robot.framePosition(self.formulation.data(), self.RF)
        self.contactRF.setReference(H_rf_ref)
        if self.conf.w_contact >= 0.0:
            self.formulation.addRigidContact(
                self.contactRF, self.conf.w_forceRef, self.conf.w_contact, 1
            )
        else:
            self.formulation.addRigidContact(self.contactRF, self.conf.w_forceRef)

        self.contact_RF_active = True

    def add_contact_LF(self, transition_time=0.0):
        H_lf_ref = self.robot.framePosition(self.formulation.data(), self.LF)
        self.contactLF.setReference(H_lf_ref)
        if self.conf.w_contact >= 0.0:
            self.formulation.addRigidContact(
                self.contactLF, self.conf.w_forceRef, self.conf.w_contact, 1
            )
        else:
            self.formulation.addRigidContact(self.contactLF, self.conf.w_forceRef)

        self.contact_LF_active = True
    
    def get_LF_3d_pos_vel_acc(self, dv):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.LF)
        v = self.robot.frameVelocity(data, self.LF)
        a = self.leftFootTask.getAcceleration(dv)
        return H.translation, v.linear, a[:3]

    def get_RF_3d_pos_vel_acc(self, dv):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.RF)
        v = self.robot.frameVelocity(data, self.RF)
        a = self.rightFootTask.getAcceleration(dv)
        return H.translation, v.linear, a[:3]