#!/usr/bin/python3
from exampleHelpers import *

sqp_solver_methods = [SQPSolverMethods.N, SQPSolverMethods.S]#["N", "S", "PCG-J", "PCG-BJ", "PCG-SS"]
mpc_solver_methods = [MPCSolverMethods.QP_PCG_BJ] #["iLQR", "QP-N", "QP-S", "QP-PCG-J", "QP-PCG-BJ", "QP-PCG-SS"]

plant = URDFPlant(options={"path_to_urdf":"../panda.urdf"})

# N is the number of timesteps
N = 50
dt = 0.1

Q = np.diag([100.0]*12)
QF = np.diag([100.0]*12)
R = np.diag([0.1]*6)
# xgoal?
xg = np.array([-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442,
               0, 0, 0, 0, 0, 0]) # because we want the speed to be 0
cost = QuadraticCost(Q,QF,R,xg)

hard_constraints = TrajoptConstraint(plant.get_num_pos(),plant.get_num_vel(),plant.get_num_cntrl(),N)
hard_constraints.set_torque_limits([30.0],[-30.0],"ACTIVE_SET")

soft_constraints = TrajoptConstraint(plant.get_num_pos(),plant.get_num_vel(),plant.get_num_cntrl(),N)
soft_constraints.set_torque_limits([30.0],[-30.0],"AUGMENTED_LAGRANGIAN")

options = {
    "expected_reduction_min_SQP_DDP":-100, # needed for hard_constraints - TODO debug why
    "display": True
}

runSQPExample(plant, cost, hard_constraints, soft_constraints, N, dt, sqp_solver_methods, options)

#runMPCExample(plant, cost, hard_constraints, soft_constraints, N, dt, mpc_solver_methods, options)




