Whole body control assignment

Use 3R Planar manipulator (uniform mass rods of mass m_i and legth L_i). Torque tau_i is applied in each joint.

Design a control system which:
- realizes circular motion of the manipulator's end effector
- realizes circular motion with minimal sum of tau_i^2 
- realizes circular motion with torque limits |tau_i| < tau_i_max

Installation:

1) Use Linux/Ubuntu

2) Install Crocoddyl and its Python bindings:

    sudo apt install robotpkg-py3*-crocoddyl
3)  Install pinocchio 

    https://stack-of-tasks.github.io/pinocchio/download.html
    
    (can try pip install pin)
3) 

    Configure your environment variables:
    
    export PATH=/opt/openrobots/bin:$PATH
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
    export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
    export PYTHONPATH=/opt/openrobots/lib/python3.*/site-packages:$PYTHONPATH

4) Then for Pycharm you can add to configuration next environment variable:

    PYTHONPATH=/opt/openrobots/lib/python3.*/site-packages
5)  Run main.py

What was done:
1) Created URDF file for 3DOF  with joint constraints
2) Realized control scheme via TSID in PyBullet (position and torque control)
3) Realized whole trajectory control optimization via Crocoddyl library with open-loop testing in PyBullet  (position and torque control)
4) Added comparison plots of torques with squared sum of torques and end effector tracking error.

Resume:

- We can see that TSID can track trajectory good and have better overall squared sum of applied torques in such type of task.

- Interesting that in pybullet position control requires less torques that torque control =)

- Still have no working solution for Crocoddyl -> Pybullet torque control scheme (need more time).

Further steps:
1) Align Crocoddyl and PyBullet dynamics models, maybe tune cost weights for working torque control. Or just use tsid for Crocoddyl trajectory execution (not needed in simple predefined circular motion).
2) Reorganize code structure to reduce doubling of pybullet simulation code.