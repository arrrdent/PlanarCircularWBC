import signal
import time

import pinocchio

import numpy as np

import crocoddyl
import pybullet as p

from target_trajectory import circle_traj_point

from numpy.linalg import norm as norm


def crocoddyl_solution(WITH_ORI=0, WITHDISPLAY=0):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Loading URDF
    manipulator = pinocchio.RobotWrapper.BuildFromURDF("three_link.urdf", [])  # abs_path, [modelPath])

    model = manipulator.model

    state = crocoddyl.StateMultibody(model)
    # Creating the shooting problem and the FDDP solver
    T = 1000
    x0 = np.array([0, 0, 1.57, 0, 0, 0])  # np.zeros(6)

    xRegCost = crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelState(state))
    uRegCost = crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelControl(state))

    # Running and terminal action models
    DT = 1e-3

    runningModels = []
    goalTrackingCosts = []

    for k in range(T):
        # Create the cost functions
        target = circle_traj_point(k * DT)

        if WITH_ORI:
            framePlacementResidual = crocoddyl.ResidualModelFramePlacement(
                state, model.getFrameId("end_link"), pinocchio.SE3(np.eye(3), target)
            )
            goalTrackingCost = crocoddyl.CostModelResidual(state, framePlacementResidual)
        else:
            # without ori
            frameTranslationResidual = crocoddyl.ResidualModelFrameTranslation(
                state, model.getFrameId("end_link"), target
            )
            goalTrackingCost = crocoddyl.CostModelResidual(state, frameTranslationResidual)

        goalTrackingCosts.append(goalTrackingCost)
        runningCostModel = crocoddyl.CostModelSum(state)

        runningCostModel.addCost("gripperPose", goalTrackingCost, 1e3)
        runningCostModel.addCost("stateReg", xRegCost, 1e-3)
        runningCostModel.addCost("ctrlReg", uRegCost, 1e-3)

        actuationModel = crocoddyl.ActuationModelFull(state)
        runningModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(
                state, actuationModel, runningCostModel
            ),
            DT,
        )

        runningModels.append(runningModel)

    # # Create cost model per each action model
    # terminalCostModel = crocoddyl.CostModelSum(state)
    #
    # # Then let's added the running and terminal cost functions
    # terminalCostModel.addCost("gripperPose", goalTrackingCosts[-1], 1e5)
    # terminalCostModel.addCost("stateReg", xRegCost, 1e-4)
    # terminalCostModel.addCost("ctrlReg", uRegCost, 1e-5)
    #
    # terminalModel = crocoddyl.IntegratedActionModelEuler(
    #     crocoddyl.DifferentialActionModelFreeFwdDynamics(
    #         state, actuationModel, terminalCostModel
    #     ),
    #     0.0,
    # )

    problem = crocoddyl.ShootingProblem(x0, runningModels, runningModels[-1])  # terminalModel)

    # solver = crocoddyl.SolverFDDP(problem)
    # have solution convergence troubles with BoxFDDP solver and bounded task (due feasibility reasons)
    # so use BoxDDP or DDP
    solver = crocoddyl.SolverFDDP(problem)

    cameraTF = [1.4, 0.0, 0.2, 0.5, 2.5, 2.5, 0.5]
    if WITHDISPLAY:
        display = crocoddyl.MeshcatDisplay(manipulator)
    solver.setCallbacks([crocoddyl.CallbackVerbose()])

    solver.getCallbacks()[0].precision = 3
    solver.getCallbacks()[0].level = crocoddyl.VerboseLevel._2

    solver.setCallbacks(
        [
            crocoddyl.CallbackVerbose(),
            crocoddyl.CallbackLogger(),
        ]
    )
    # Solving the problem
    solver.solve([], [], 1000)

    # collect data from solver
    xs, us, accs, fs = [], [], [], []
    knots = 0

    s = solver
    knots += len(s.problem.runningModels)
    models = s.problem.runningModels.tolist()
    datas = s.problem.runningDatas.tolist()
    for i, data in enumerate(datas):
        xs.append(s.xs[i])  # state
        us.append(s.us[i])  # control
    print("Total knots:" + str(knots))

    # Display the entire motion
    if WITHDISPLAY:
        display.rate = -1
        display.freq = 1
        for i in range(1):
            display.displayFromSolver(solver)

    return xs, us


def crocoddyl_pybullet_test(xs, us, control_type=p.POSITION_CONTROL, posGain=0.15):
    dt = 0.001
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

    q0 = [0, 0, 1.57]
    for j in range(3):
        p.resetJointState(robot_pybullet, j, q0[j])

    log_pybullet_torques, ee_errors = [], []

    if control_type == p.POSITION_CONTROL:
        t = 0
        for x in xs:
            for j in range(3):
                p.setJointMotorControl2(robot_pybullet, j,
                                        p.POSITION_CONTROL,
                                        targetPosition=x[j],
                                        positionGain=posGain,
                                        force=20)
            p.stepSimulation()
            applied_torques = []
            for j in range(3):
                applied_torques.append(p.getJointState(robot_pybullet, j)[3])
            log_pybullet_torques.append(applied_torques)
            ee_pos = p.getLinkState(robot_pybullet, 3)[0]
            ee_pos_err = circle_traj_point(t) - ee_pos
            ee_errors.append(norm(ee_pos_err, 2))
            t += dt
            time.sleep(dt)

    elif control_type == p.TORQUE_CONTROL:
        p.setJointMotorControlArray(
            robot_pybullet,
            np.arange(3),
            p.VELOCITY_CONTROL,
            forces=np.zeros(3),
        )

        t = 0
        for u in us:
            for j in range(3):
                p.setJointMotorControl2(robot_pybullet, j,
                                        p.TORQUE_CONTROL,
                                        force=u[j])

            p.stepSimulation()
            log_pybullet_torques.append(u)
            ee_pos = p.getLinkState(robot_pybullet, 3)[0]
            ee_pos_err = circle_traj_point(t) - ee_pos
            ee_errors.append(norm(ee_pos_err, 2))
            t += dt
            time.sleep(dt)

    p.resetSimulation()
    p.disconnect()
    return {"torques": np.array(log_pybullet_torques), "ee_tracking_errors": ee_errors}
