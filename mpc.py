import numpy as np
import matplotlib.pyplot as plt
from cvxopt import matrix,solvers
def main():
    x0 = np.array([[100],[4]])
    dt = 0.1
    v_des = 7.0
    #* Distance and time constraint for a cell
    # cell = (sInCell,TinCell)
    # From egoX T received as time from current timestep
    CellList = [ (110,2),(104,1)] 
    qpMPC(x0,dt,v_des,CellList,1)


def qpMPC(x0,dt,v_des,CellList,ID,N=20, inputCosts = 3.5, devCosts = 2):
    #TODO fix bug, always uses max input?
    #TODO make into class and separate into setup and optimize and reuse some matrices
    # Finite horizon
    # x0 = matrix(np.array([egoX.sTraversed],[egoX.vel_norm]))
    # Along path, so control is sdotdot, states are s and sdot

    #* State space:
    # ss_A = [1,dt;0,1]
    # ss_B = [0;1]
    # x(k) = [s(k);v(k)]
    # x(k+1) = [s(k+1);v(k+1)] = ss_A*x(k) + ss_B*u(k)
    ss_A = np.array([[1,dt],[0,1]])
    ss_B = np.array([[0],[1]])
    #* Constraints
    a_max = 2.25
    a_min = 9
    u_max = a_max*dt # Maximum acceleration 
    u_min = a_min*dt # Maximum deceleration
    v_max = 10 # Maximum velocity
    v_min = 0 # Minimum Velocity
    
    N_slack = 0
    for cell in CellList:
        sIn = cell[0]
        tIn = cell[1]
        if tIn > 0 and tIn < N*dt:
            N_slack += 1
    if N_slack == 0:
        N_slack = 1


    #* Constraints Matrix Stacking
    #! Help matrices, not actual matrices
    states = 2 # Amount of actual state in the system
    inputs = 1 # Amount of inputs in the system
    x_vec = np.zeros((states*N,1))      # states have no costs
    u_vec = np.add(np.ones((N,1)),np.arange(0,1,1/N).reshape(N,1)*3)  # input cost shape
    u_vec = inputCosts*u_vec/np.linalg.norm(u_vec)*inputCosts # Apply input cost to normalized vector
    v_ref = np.zeros((1,1))             # reference velocity has no cost
    v_dev = np.ones((N,1))*devCosts     # deviation form reference velocity has a cost
    x_slack = np.ones((1,1))*100000     # Slack variable with very high cost
    #! xi_vec also functions as the cost weights
    xi_vec = np.concatenate((x_vec,u_vec,v_ref,v_dev,x_slack),axis=0)    
    statevector = ['x[%d](%d)' % (i,j) for i in range(N) for j in range(states)] + ['u[%d]' % i for i in range(len(u_vec))] + ['v_ref']+['v_dev[%d]' % i for i in range(N) ] + ['slck[%d]' % i for i in range(N_slack)]
    N_x = states*N 
    N_u = inputs*N 
    N_v_ref = 1 # states added vor v_ref 
    N_v_dev = N # state added for v_dev
    N_tot = N_x+N_u+N_v_ref+N_v_dev+N_slack# 2 for v_ref and v_dev
    


    # Inequality constraints
    # TODO add min and max velocity constraints
    # Indicator Matrices for inputs
    Aieq_u_max = np.concatenate( ( np.zeros( (N,N_x) ) , np.eye(N_u), np.zeros( (N,N_tot-(N_x+N_u)) ) ) , axis=1 )
    bieq_u_max = np.repeat( np.array([[u_max]]) , N_u , axis=0 )
    Aieq_u_min = np.concatenate( ( np.zeros( (N,N_x) ) , -np.eye(N_u), np.zeros( (N,N_tot-(N_x+N_u)) ) ) , axis=1 )
    bieq_u_min = np.repeat( np.array([[u_min]]) , N_u , axis=0 )
    
    bieq_v_max = np.repeat( np.array([[v_max]]) , N_u , axis=0 )
    bieq_v_min = np.repeat( np.array([[v_min]]) , N_u , axis=0 )
    Aieq_v_max = np.zeros((0,N_tot))
    Aieq_v_min = np.zeros((0,N_tot))
    for i in range(N):
        Aieq_v_max = np.concatenate((Aieq_v_max,np.concatenate( ( np.zeros( (1,i*states+1) ) , np.ones((1,1)), np.zeros( (1,N_tot-(i+1)*states) ) ) , axis=1 )), axis=0)
        Aieq_v_min = np.concatenate((Aieq_v_min,np.concatenate( ( np.zeros( (1,i*states+1) ) , -np.ones((1,1)), np.zeros( (1,N_tot-(i+1)*states) ) ) , axis=1 )), axis=0)
    
    
    # Time constraint implemented as state constraint for displacement at certain timestep
    Aieq_st = np.zeros((0,N_tot))
    bieq_st = np.zeros((0,1))

    tIndex = None
    slackIndex = 0 
    for cell in CellList:
        sIn = cell[0]
        tIn = cell[1]
        if tIn > 0 and tIn < N*dt:           
            tIndex = int(np.floor(tIn/dt))
            Aieq_st = np.concatenate( (Aieq_st,np.concatenate( (np.zeros((1, states*(tIndex))), np.array([[1, 0]]), np.zeros((1,N_tot-(tIndex+1)*states-(N_slack-slackIndex))),np.ones((1,1)),np.zeros((1,N_slack - 1 - slackIndex)) ), axis = 1)) , axis = 0)
            bieq_st = np.concatenate( (bieq_st,np.array([[sIn]]) ), axis = 0)
            slackIndex += 1
            
        elif tIn < 0 and tIndex == None:
            tIndex = 0
    if tIndex == None:
        tIndex = N-1
        # If none of the cells are within finite horizon, add a constraint at last timestep for interpolated distance, to avoID speeding through.
        # sIn at first (ds/t)*(N*dt), average velocity needed to reach tIn* time at end of finite horizon
        sIn = x0[0][0]+(CellList[0][0]-x0[0][0])/(CellList[0][1])*((N-1)*dt)  #+0.125 !REMOVED FOR NOW Relaxed with some time in order to prevent tIn increasing itself perpetually
        Aieq_st = np.concatenate( (Aieq_st,np.concatenate( (np.zeros((1, states*(tIndex))), np.array([[1, 0]]), np.zeros((1,N_tot-(tIndex+1)*states-(N_slack-slackIndex))),np.ones((1,1)),np.zeros((1,N_slack - 1 - slackIndex)) ), axis = 1)) , axis = 0)
        bieq_st = np.concatenate( (bieq_st,np.array([[sIn]]) ), axis = 0)



    
    # printFormulaIeq(Aieq_st,bieq_st,statevector)
    # printFormulaIeq(Aieq_v_max,bieq_v_max,statevector)
    # printFormulaIeq(Aieq_v_min,bieq_v_min,statevector)

    Aieq = np.concatenate( ( Aieq_u_max,Aieq_u_min,Aieq_v_min,Aieq_v_max,Aieq_st ) , axis=0 )
    bieq = np.concatenate( ( bieq_u_max,bieq_u_min,bieq_v_min,bieq_v_max,bieq_st ) , axis=0 )


    
    # Equality constraints
    Aeq_v_ref = np.concatenate( (np.zeros((1,N_x+N_u)),np.array([[1]]),np.zeros((1,N_tot-(N_x+N_u+N_v_ref)))) , axis = 1 ) 
    beq_v_ref = np.ones((1,1))*v_des
    Aeq_v_dev = np.zeros((0,N_tot))
    beq_v_dev = np.zeros((N,1))
    Aeq_ss = np.zeros((0,N_tot))
    beq_ss = np.concatenate((x0 , np.zeros((N_x-states,1))) , axis = 0)
    ss_Bu = np.zeros((states,0))
    # TODO make more efficient maybe, depends on tic toc, concatenate is not efficient apparently. ss_Ax,ss_Bu,ss_Ab can be done in one call.
    for i in range(N):
        if i > 0:
            # -np.eye because Ax0 - Ix1 + Bu0 = 0 => x1 = Ax0 + Bu0
            ss_Ax = np.concatenate( (np.linalg.matrix_power(ss_A, i),np.zeros((states, (i-1)*states)), -np.eye(states), np.zeros((states, N_x-(i+1)*states))) ,  axis =1)
            # ss_Bu = np.concatenate( (np.zeros((states,N_u+N_tot-(N_x+N_u))),np.zeros((states,0))),axis =1)
            ss_Bu = np.concatenate( (np.matmul(np.linalg.matrix_power(ss_A, i-1), ss_B), ss_Bu[:,0:(i-1)*inputs], np.zeros((states, N_u+N_tot-(N_x+N_u)-(i)*inputs)) ) , axis=1)
            ss_AB = np.concatenate( (ss_Ax,ss_Bu) , axis = 1)
        else:
            ss_AB = np.concatenate( (np.eye(states),np.zeros((states,N_tot-states))),axis =1)
        # TODO finish this with v dev being one for each timestep now
        # Doesn't scale on states, as np.array([[0,1]]) is hard coded
        Aeq_v_dev_i = np.concatenate( (np.zeros((1,states*i)),np.array([[0,1]]),np.zeros((1,(N_x+N_u)-states*(i+1))),np.array([[-1]]),np.zeros((1,i)),np.array([[-1]]),np.zeros((1,N_tot-(N_x+N_u+N_v_ref+(i+1))))), axis = 1 ) 
        Aeq_ss = np.concatenate((Aeq_ss,ss_AB) , axis = 0 )
        Aeq_v_dev = np.concatenate( (Aeq_v_dev,Aeq_v_dev_i), axis = 0 ) 
    Aeq = np.concatenate( ( Aeq_ss,Aeq_v_ref,Aeq_v_dev) , axis=0 )
    beq = np.concatenate( ( beq_ss,beq_v_ref,beq_v_dev) , axis=0 )

    #* Cost function matrices
    P = np.diagflat(xi_vec)
    q = np.zeros((N_tot,1))

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
    solvers.options['show_progress'] = False
    sol = solvers.qp(P,q,G,h,A,b)
    if sol['status'] != 'optimal':
        # plotStuff(sol,N,N_x,N_u,inputCosts,devCosts,CellList,dt,ID,sIn)
        print("infeasible")
    u0 = sol['x'][N_x:N_x+inputs]
    # u = sol['x'][N_x:N_x+N_u]
    # print(u)
    # print(sol['x'])
    # plotStuff(sol,N,N_x,N_u,inputCosts,devCosts,CellList,dt,ID,sIn,1)
    # printFormula(Aeq_ss,statevector)
    # printFormulaEq(Aeq,beq,statevector)
    # printFormulaIeq(Aieq,bieq,statevector)
    # printFormula(np.array(P),statevector)
    # import matplotlib.pyplot as plt
    # plt.pcolor(P,cmap = 'gray')
    # plt.gca().invert_yaxis()
    # plt.show()
    return (sol,u0)


def plotStuff(sol,N,N_x,N_u,inputCosts,devCosts,CellList,dt,ID,sInN=None,onTick=None):
    splot = []
    vplot = []
    uplot = []
    vdevplot = []
    Jplot = []
    for i in range(N):
        splot.append(sol['x'][i*2])
        vplot.append(sol['x'][i*2+1])
        uplot.append(sol['x'][N_x+i])
        vdevplot.append(sol['x'][N_x+N_u+1+i])
        Jplot.append(inputCosts*sol['x'][N_x+i]**2+devCosts*sol['x'][N_x+N_u+1+i]**2)
    import matplotlib.pyplot as plt
    fig,axs = plt.subplots(2,1,num=0)
    axs[0].cla()
    axs[1].cla()
    axs[1].plot(splot,'x')
    axs[1].plot(vplot,'x')
    axs[1].plot(uplot,'x')
    axs[1].plot(vdevplot,'x')
    axs[0].plot(Jplot,'x')
    tIndex = None
    for cell in CellList:
        sIn = cell[0]
        tIn = cell[1]
        if tIn > 0 and tIn < N*dt:
            tIndex = np.floor(tIn/dt)
            axs[1].plot(tIndex,sIn,'ro')
    if tIndex == None and sIn != None:
        axs[1].plot(N-1,sInN,'go')
    axs[0].legend(('cost'))
    axs[1].legend(('s','v','u','vdev','constraints'))
    # label = "Finite horizon (k) [N= "+str(N)+",dt = "+ str(dt)+"]"
    # axs[1].set_xlabel(label)
    fig.suptitle(str(ID))
    if onTick == 1:
        plt.show(block=False)
        plt.pause(0.001)
    else:
        plt.show()




def printFormula(array,statevector):
    for row in array:
        line = ''
        for column in enumerate(row):
            if column[1] != 0:
                if column[1] > 0:
                    line = line + '+' +str(column[1]) + '*' + str(statevector[column[0]]) + ' '
                else:
                    line = line + str(column[1]) + '*' + str(statevector[column[0]]) + ' '
        print(line)

def printFormulaEq(Aeq,beq,statevector):
    for row in enumerate(Aeq):
        line = ''
        for column in enumerate(row[1]):
            if column[1] != 0:
                if column[1] > 0:
                    line = line + '+' +str(column[1]) + '*' + str(statevector[column[0]]) + ' '
                else:
                    line = line + str(column[1]) + '*' + str(statevector[column[0]]) + ' '
        line = line + '= ' + str(beq[row[0]][0])
        print(line)

def printFormulaIeq(Aieq,bieq,statevector):
    for row in enumerate(Aieq):
        line = ''
        for column in enumerate(row[1]):
            if column[1] != 0:
                if column[1] > 0:
                    line = line + '+' +str(column[1]) + '*' + str(statevector[column[0]]) + ' '
                else:
                    line = line + str(column[1]) + '*' + str(statevector[column[0]]) + ' '
        line = line + '<=' + str(bieq[row[0]][0])
        print(line)

if __name__ == '__main__':
    main()