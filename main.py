import pybullet as p

from crocoddyl_solution import crocoddyl_pybullet_test, crocoddyl_solution
from plotting import plotTorques
from tsid_solution import tsid_pybullet_test

if __name__ == '__main__':
    # Manipulator End-Effector Circle Tracking tests

    results = {}
    # collect applied torques for tsid position and torque control in pybullet
    results["tsid_pos_ctrl"] = tsid_pybullet_test(POS_CTRL=1)
    results["tsid_torq_ctrl"] = tsid_pybullet_test(POS_CTRL=0)

    # solve ddp task for states and control toques
    xs, us = crocoddyl_solution()
    # apply states (position control) and control (torque control) in pybullet
    results["croc_pos_ctrl"] = crocoddyl_pybullet_test(xs, us, control_type=p.POSITION_CONTROL)
    results["croc_torq_ctrl"] = crocoddyl_pybullet_test(xs, us, control_type=p.TORQUE_CONTROL)

    plotTorques(results)
