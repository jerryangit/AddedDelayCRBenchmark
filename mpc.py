import numpy as np
import matplotlib.pyplot as plt
from cvxopt import matrix,solvers
def main():

    # From EgoX:
    v_des = 5 # desired velocity
    dt = 0.01 # dt

    # Along path, so control is sdotdot, states are s and sdot
    # Finite horizon
    N = 5
    #* State space:
    # ss_A = [1,dt;0,1]
    # ss_B = [0;1]
    # x(k) = [s(k);v(k)]
    # x(k+1) = [s(k+1);v(k+1)] = ss_A*x(k) + ss_B*u(k)
    ss_A = np.array([[1,dt],[0,1]])
    ss_B = np.array([[0],[1]])
    #* Constraints
    a_max = 1
    a_min = -1
    u_max = a_max*dt # Maximum acceleration 
    u_min = a_min*dt # Maximum deceleration
    v_max = 10 # Maximum velocity
    v_min = 0 # Minimum Velocity
    # Time constraint implemented as state constraint for displacement at certain timestep
    Tin = 2 #Ingoing time 
    # x0 = matrix(np.array([egoX.sTraversed],[egoX.vel_norm]))
    x0 = [0,0]
    #* Constraints Matrix Stacking
    #! Help matrices, not actual matrices
    states = 2 # Amount of actual state in the system
    inputs = 1 # Amount of inputs in the system
    x_vec = np.zeros((states*N,1))  # states have no costs
    u_vec = np.ones((N,1))          # input cost vector
    v_ref =  np.zeros((1,1))        # reference velocity has no cost
    v_dev = np.ones((N,1))          # deviation form reference velocity has a cost
    xi_vec = np.concatenate((x_vec,u_vec,v_ref,v_dev),axis=0)
    statevector = ['x[%d](%d)' % (i,j) for i in range(N) for j in range(states)] + ['u[%d]' % i for i in range(len(u_vec))] + ['v_ref']+['v_dev[%d]' % i for i in range(N)]
    N_x = states*N 
    N_u = inputs*N 
    N_v_ref = 1 # states added vor v_ref 
    N_v_dev = N # state added for v_dev
    N_tot = N_x+N_u+N_v_ref+N_v_dev# 2 for v_ref and v_dev
    # Inequality constraints
    # Indicator Matrices for inputs
    Aieq_u_max = np.concatenate( ( np.zeros( (N,N_x) ) , np.eye(N_u), np.zeros( (N,N_v_ref+N_v_dev) ) ) , axis=1 )
    Aieq_u_min = np.concatenate( ( np.zeros( (N,N_x) ) , -np.eye(N_u), np.zeros( (N,N_v_ref+N_v_dev) ) ) , axis=1 )
    bieq_u_max = np.repeat( u_max , N_u , axis=0 )
    bieq_u_min = np.repeat( u_min , N_u , axis=0 )
    Aieq = np.concatenate( ( Aieq_u_max,Aieq_u_min ) , axis=0 )
    bieq = np.concatenate( ( bieq_u_max,bieq_u_min ) , axis=0 )
    # Equality constraints
    Aeq_v_ref = np.concatenate( (np.zeros((1,N_x+N_u)),np.array([[1]]),np.zeros((1,N_v_dev))) , axis = 1 ) 
    beq_v_ref = np.ones((1,1))*v_des
    Aeq_v_dev = np.zeros((0,N_tot))
    beq_v_dev = np.zeros((1,1))
    Aeq_ss = np.zeros((0,N_tot))
    beq_ss = np.zeros((N_x,1))
    # TODO make more efficient maybe, depends on tic toc, concatenate is not efficient apparently. ss_Ax,ss_Bu,ss_Ab can be done in one call.
    for i in range(N):
        if i > 0:
            # -np.eye because Ax0 - Ix1 + Bu0 = 0 => x1 = Ax0 + Bu0
            ss_Ax = np.concatenate( (np.linalg.matrix_power(ss_A,i),np.zeros((states,(i-1)*states)),-np.eye(states),np.zeros((states,N_x-(i+1)*states))) ,  axis =1)
            ss_Bu = np.concatenate( (np.zeros((states,N_u+N_v_ref+N_v_dev)),np.zeros((states,0))),axis =1)
            ss_AB = np.concatenate((ss_Ax,ss_Bu),axis = 1)
        else:
            ss_AB = np.concatenate( (np.eye(states),np.zeros((states,N_tot-states))),axis =1)
        # TODO finish this with v dev being one for each timestep now
        # Doesn't scale on states, as np.array([[0,1]]) is hard coded
        Aeq_v_dev_i = np.concatenate( (np.zeros((1,states*i)),np.array([[0,1]]),np.zeros((1,(N_x+N_u)-states*(i+1))),np.array([[-1]]),np.zeros((1,i)),np.array([[-1]]),np.zeros((1,N_v_dev-(i+1)))), axis = 1 ) 
        Aeq_ss = np.concatenate((Aeq_ss,ss_AB) , axis = 0 )
        Aeq_v_dev = np.concatenate( (Aeq_v_dev,Aeq_v_dev_i), axis = 0 ) 
    printFormula(Aeq_ss,statevector)
    printFormula(Aeq_v_dev,statevector)

    Aeq = np.concatenate( ( Aeq_ss,Aeq_v_ref,Aeq_v_dev) , axis=0 )
    beq = np.concatenate( ( beq_ss,beq_v_ref,beq_v_dev) , axis=0 )
    #* Cost function matrices
    P = np.diag([])
    q = np.array([[0],[0],[0]])

    #* Optimization
    # 1/2 xT P x + qx
    # s.t.  G * x <= h
    #       A * x = b
    # for k in range(optim_iter):
    P = matrix(P, tc = 'd')
    q = matrix(q, tc = 'd')
    G = matrix(Aieq, tc = 'd') 
    h = matrix(bieq, tc = 'd')
    A = matrix(Aeq, tc = 'd')
    b = matrix(beq, tc = 'd')
    sol = solvers.qp(P,q,G,h,A,b)
    u = None

def printFormula(matrix,statevector):
    for row in matrix:
        line = ''
        for column in enumerate(row):
            if column[1] != 0:
                if column[1] > 0:
                    line = line + '+' +str(column[1]) + '*' + str(statevector[column[0]]) + ' '
                else:
                    line = line + str(column[1]) + '*' + str(statevector[column[0]]) + ' '
        print(line)

if __name__ == '__main__':
    main()