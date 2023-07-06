import time as tm

import numpy as np
import pybullet as p
from numpy import nan
from numpy.linalg import norm as norm

import tsid_manipulator_conf as tsid_conf
from target_trajectory import circle_traj_point
from tsid_maniplulator import TsidManipulator


def tsid_pybullet_test(POS_CTRL=0, posGain=0.15):
    dt = tsid_conf.dt
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=45, cameraPitch=-30,
                                 cameraTargetPosition=[0, 0, 0.5])
    p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
    p.setGravity(0, 0, 9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
    robot_pybullet = p.loadURDF("three_link.urdf", basePosition=[0, 0, 0], useFixedBase=1)
    p.changeDynamics(robot_pybullet, -1, linearDamping=0.04, angularDamping=0.04)
    p.setRealTimeSimulation(0)
    p.setTimeStep(dt)
    for j in range(3):
        p.resetJointState(robot_pybullet, j, tsid_conf.q0[j])

    tsid = TsidManipulator(tsid_conf)

    N = tsid_conf.N_SIMULATION

    log_pybullet_torques = np.empty((tsid.robot.na, N)) * nan

    tau = np.empty((tsid.robot.na, N)) * nan
    q = np.empty((tsid.robot.nq, N + 1)) * nan
    v = np.empty((tsid.robot.nv, N + 1)) * nan
    ee_pos = np.empty((3, N)) * nan
    ee_vel = np.empty((3, N)) * nan
    ee_acc = np.empty((3, N)) * nan
    ee_pos_ref = np.empty((3, N)) * nan
    ee_vel_ref = np.empty((3, N)) * nan
    ee_acc_ref = np.empty((3, N)) * nan
    ee_acc_des = np.empty((3, N)) * nan  # acc_des = acc_ref - Kp*pos_err - Kd*vel_err

    # end effector error
    ee_errors = np.empty(N) * nan

    sampleEE = tsid.trajEE.computeNext()

    pEE = sampleEE.value().copy()
    vEE = np.zeros(3)
    aEE = np.zeros(3)

    t = 0.0
    q[:, 0], v[:, 0] = tsid.q, tsid.v

    for i in range(0, N):
        time_start = tm.time()

        pEE[:3] = circle_traj_point(i * dt)
        # we can use future information about our trajectory
        # or set velocity and acceleration to zeros for unpredictable trajectory
        vEE[:3] = (circle_traj_point((i + 1) * dt) - circle_traj_point(i * dt)) / dt  # np.zeros(3)
        aEE[:3] = ((circle_traj_point((i + 2) * dt) - circle_traj_point((i + 1) * dt)) / dt -
                   (circle_traj_point((i + 1) * dt) - circle_traj_point(i * dt)) / dt) / dt  # np.zeros(3)
        sampleEE.value(pEE)  # pos
        sampleEE.derivative(vEE)  # vel
        sampleEE.second_derivative(aEE)  # acc
        tsid.eeTask.setReference(sampleEE)

        HQPData = tsid.formulation.computeProblemData(t, q[:, i], v[:, i])

        sol = tsid.solver.solve(HQPData)
        if sol.status != 0:
            print(("Time %.3f QP problem could not be solved! Error code:" % t, sol.status))
            break

        tau[:, i] = tsid.formulation.getActuatorForces(sol)

        dv = tsid.formulation.getAccelerations(sol)

        ee_pos[:, i] = tsid.robot.framePosition(
            tsid.formulation.data(), tsid.EE
        ).translation
        ee_vel[:, i] = tsid.robot.frameVelocityWorldOriented(
            tsid.formulation.data(), tsid.EE
        ).linear
        ee_acc[:, i] = tsid.eeTask.getAcceleration(dv)[:3]
        ee_pos_ref[:, i] = sampleEE.value()[:3]
        ee_vel_ref[:, i] = sampleEE.derivative()[:3]
        ee_acc_ref[:, i] = sampleEE.second_derivative()[:3]
        ee_acc_des[:, i] = tsid.eeTask.getDesiredAcceleration[:3]

        # time to switch to torque control
        switch_torq_i = 0
        if POS_CTRL:
            # for POS CTRL need integrate dv
            q[:, i + 1], v[:, i + 1] = tsid.integrate_dv(q[:, i], v[:, i], dv, tsid_conf.dt)

            # apply control to pybullet
            for j in range(3):
                p.setJointMotorControl2(robot_pybullet, j,
                                        p.POSITION_CONTROL,
                                        targetPosition=q[j, i + 1],
                                        positionGain=posGain,
                                        force=tsid.model.effortLimit[j])
        else:
            if i == switch_torq_i:
                # Disable the velocity control on the joints as we use torque control.
                p.setJointMotorControlArray(
                    robot_pybullet,
                    np.arange(3),
                    p.VELOCITY_CONTROL,
                    forces=np.zeros(3),
                )
            # for TORQ CTRL need get real q, v from pybullet
            joint_states = p.getJointStates(robot_pybullet, np.arange(3))
            q_pb, v_pb = np.zeros(tsid.robot.nq), np.zeros(tsid.robot.nq)
            for j in range(3):
                q_pb[j] = joint_states[j][0]
                v_pb[j] = joint_states[j][1]
            q[:, i + 1], v[:, i + 1] = q_pb.copy(), v_pb.copy()

            # apply control to pybullet
            for j in range(3):
                p.setJointMotorControl2(robot_pybullet, j,
                                        p.TORQUE_CONTROL,
                                        force=tau[j, i])

        # or just use tsid.eeTask.position_error
        ee_pos_pb = p.getLinkState(robot_pybullet, 3)[0]
        ee_pos_err = circle_traj_point(t) - ee_pos_pb

        ee_errors[i] = norm(ee_pos_err, 2)

        t += tsid_conf.dt
        p.stepSimulation()

        log_pybullet_torques[:, i] = tau[:, i]

        time_spent = tm.time() - time_start
        if time_spent < tsid_conf.dt:
            tm.sleep(tsid_conf.dt - time_spent)

    p.disconnect()
    return {"torques": np.array(log_pybullet_torques).T, "ee_tracking_errors": ee_errors}
