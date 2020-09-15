import osqp
import numpy as np
import scipy as sp
import copy
import sys
import matplotlib.pyplot as plt
from scipy import sparse

def main():
    dt = 0.1
    d_min = 3.5
    d_mult = 2.25
    rho_base = 1
    phi_a = 6
    mu_0 = 0.25
    N = 10                     # Prediction horizon
    mcN_Dist = 25              # Distance at vehicle is added to mcN
    mpc = oa_mpc(dt,N,d_min,d_mult)
    egoX = egoXDummy()
    xDestLoc = np.array([10,1,egoX.velRef])
    x_ref =  np.linspace([0,0,egoX.velRef],xDestLoc,N+1).flatten()
    mcN = set([1,2,3])
    z_JI = {}
    x_J = {}
    lambda_JI = {}
    rho_JI = {}
    lambda_JI[1] = np.ones(2*N)
    lambda_JI[2] = np.ones(2*N)
    lambda_JI[3] = np.ones(2*N)    
    rho_JI[1] = np.ones(2*N)
    rho_JI[2] = np.ones(2*N)
    rho_JI[3] = np.ones(2*N)    
    z_JI[1] = np.linspace([0,0],xDestLoc[:2] + np.array((-4,1)),N).flatten()
    z_JI[2] = np.linspace([0,0],xDestLoc[:2] + np.array((-4,1)),N).flatten()
    z_JI[3] = np.linspace([0,0],xDestLoc[:2] + np.array((-4,1)),N).flatten()    
    mpc.setupMPC_x(egoX)    
    I_xyv = sparse.kron(np.vstack([np.zeros((1,self.N)),np.eye(self.N)]),np.array(([1., 0.], [0., 1.],[0.,0.]))) # Indicator matrix to get a vector with only XY coordinates shape of 2*N        
    (res,ctrl,xTraj) = mpc.solveMPC_x()
    # plt.plot(xTraj[::2])
    # plt.plot(xTraj[1::2])    
    # plt.plot(res.x[2:33:3])    
    # plt.legend(('x','y','v'))
    # plt.show()
    x_J[1] = xTraj
    x_J[2] = xTraj  + np.linspace([10,-2], [1,-2],N).flatten()
    x_J[3] = xTraj   + np.linspace([-10,0], [2,0],N).flatten()
    mpc.setupMPC_z(egoX,mcN,lambda_JI, rho_JI, x_J)    
    # mpc.updateMPC_z(egoX,x_ref,z_JI,lambda_JI,rho_JI,mcN)
    (res) = mpc.solveMPC_z()


class egoXDummy:
    def __init__(self):
        self.velRef = 8.33
        self.velNorm = 6
        self.rotation =rotationYawDummy(15)

class rotationYawDummy:
    def __init__(self,yaw):
        self.yaw = yaw

class oa_mpc:
    def __init__(self,dt,N,d_min,d_mult):
        # Pass through variables           
        self.N = N
        self.dt = dt 
        self.d_min = d_min
        self.d_mult = d_mult
        # Vehicle porperties
        self.deltaMax = (np.pi*4)/9
        self.ddeltaMax = self.deltaMax*2 # (self.dt/0.125)   # Maximum steering angle change per step, number is amount of seconds per full rotation

        # Not configurable
        self.nx = 3                     # Number of states
        self.nu = 2                     # Number of inputs
        self.nxc = self.nx*(self.N+1)   # Number of states concatenated
        self.nuc = self.nu*self.N       # Number of inputs concatenated
        self.nucZ = np.zeros(self.nuc)     # Empty array of zeros of shape (nuc,)
        self.delta = 1e-10                  # Initialize delta
        self.beta = 1e-10                   # Initialize beta
        self.I_xy0 = sparse.kron(np.eye(self.N+1),np.array(([1., 0., 0.], [0., 1.,0.]))) # Indicator matrix to get a vector with only XY coordinates shape of 2*(N+1)
        self.I_xy = sparse.kron(np.hstack((np.zeros((self.N,1)),np.eye(self.N))),np.array(([1., 0., 0.], [0., 1.,0.]))) # Indicator matrix to get a vector with only XY coordinates shape of 2*N        
        self.I_xyv = sparse.kron(np.vstack([np.zeros((1,self.N)),np.eye(self.N)]),np.array(([1., 0.], [0., 1.],[0.,0.]))) # Indicator matrix to get a vector with only XY coordinates shape of 2*N        
        self.ctrl = (1e-5,1e-5)

    def setupMPC_x(self,egoX):
        self.v_ref = egoX.velRef
        self.x0 = np.array((0.,0.,egoX.velRef))
        # Linearized discrete time model of a unicycle model in local coordinate frame
        # x(k+1) = Ad x(k) + Bd u(k)
        # x = [X,Y,v], u = [throttle (a), steering angle (delta)]
        # Model parameters
        # Define continuous-time linear model from Jacobian of the nonlinear model.
        Ad = sparse.csc_matrix([
            [1., 0., self.dt*np.cos(self.beta)],
            [0., 1., self.dt*np.sin(self.beta)],
            [0., 0., 1.]
        ])
        # Bd = sparse.csc_matrix([
        #     [0., -self.dt*self.v/4.],
        #     [0., self.dt*self.v/2.],
        #     [self.dt, 0.]
        # ])
        Bd = sparse.csc_matrix([
            [0., -self.dt*self.x0[2]/2+1e-10],
            [0., self.dt*self.x0[2]/2+1e-10],
            [self.dt, 0.]
        ])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(self.N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(self.N*self.nx)])
        ueq = leq

        # Input constraints 
        umin = np.array([-10, -self.deltaMax])
        umax = np.array([3.5, self.deltaMax])
        # Ineq constraints, collision avoidance
        xmin = np.array([-np.inf,-np.inf,-5])
        xmax = np.array([np.inf,np.inf,12])
        Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        lineq = np.hstack([np.kron(np.ones(self.N+1), xmin), np.kron(np.ones(self.N), umin)])
        uineq = np.hstack([np.kron(np.ones(self.N+1), xmax), np.kron(np.ones(self.N), umax)])

        # Cost function
        Q = sparse.diags([0.5, 7.5, 0.01])
        QN = Q*1
        R = sparse.diags([0.25, 10.0])
        x_ref = np.linspace([0,0,self.x0[2]],[1,1,self.v_ref],self.N+1).flatten()


        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        self.P_Q = sparse.block_diag([sparse.kron(sparse.eye(self.N), Q), QN, sparse.kron(sparse.diags(np.linspace(1.0,1.0,self.N)), R,format='csc')], format='csc') 
        P = self.P_Q        
        # - linear objective
        self.lambda_ref = np.hstack([np.kron(np.ones(self.N), Q.diagonal()),QN.diagonal()])
        q = np.hstack([np.multiply(self.lambda_ref,-x_ref), self.nucZ])         

        A = sparse.vstack([Aeq, Aineq], format='csc')
        self.l = np.hstack([leq, lineq])
        self.u = np.hstack([ueq, uineq])
        P,q 
        # Create an OSQP object
        self.prob_x = osqp.OSQP()
        # Setup workspace
        self.prob_x.setup(P, q, A, self.l, self.u, warm_start=True, polish = 1 ,verbose = 0, max_iter = 10000, scaling=20,eps_abs = 1e-5, eps_rel = 1e-3)


    def updateMPC_x(self,egoX,x_ref,z_JI,lambda_JI,rho_JI,mcN):
        # x_ref.shape = (N+1)*3, z_JI.get(id).shape = (N+1)*3, lambda_JI.get(id).shape = N*2, rho_JI.get(id).shape = N*2

        # Calculate actual beta based on previous control input
        delta = egoX.ego.get_control().steer*((4*np.pi)/9)
        self.beta = np.arctan(1/2*np.tan(delta)) + 1e-10 # Add very small amount to ensure Ad elements don't become zero 

        # Calculate the actual velocity in the direction of bicycle model
        R = np.array([[np.cos(-self.beta),-np.sin(-self.beta)],[np.sin(-self.beta),np.cos(-self.beta)]])
        egoX.velLocNorm = np.array([1,0])@R@egoX.velLoc
        self.x0 = np.array([0,0,1*egoX.velLocNorm+0.0*egoX.velNorm + 1e-10])

        # Update Q and q based on OA-ADMM rules # q = lambda.*(-reference)
        q = np.hstack([np.multiply(self.lambda_ref+self.I_xyv@rho_JI.get(egoX.id),-x_ref), self.nucZ])
        P = self.P_Q  + sparse.diags(np.hstack((self.I_xyv@rho_JI.get(egoX.id), self.nucZ)), format='csc')
        for vin in mcN:
            q_rho = np.hstack([np.multiply(self.I_xyv@rho_JI.get(vin),-z_JI.get(vin)), self.nucZ])
            q_lambda = np.hstack((1/2*self.I_xyv@lambda_JI.get(vin),self.nucZ))
            q = q + q_rho + q_lambda
            P_Rho = sparse.diags(np.hstack((self.I_xyv@rho_JI.get(vin),self.nucZ)), format='csc')
            P = P + P_Rho
        Ad = sparse.csc_matrix([
        [1., 0., self.dt*np.cos(self.beta) + 1e-10],
        [0., 1., self.dt*np.sin(self.beta) + 1e-10],
        [0., 0., 1.]
        ])
        Bd = sparse.csc_matrix([
        [0., -self.dt*self.x0[2]/2+1e-10],
        [0., self.dt*self.x0[2]/2 +1e-10],
        [self.dt, 0.]])
        Ax = sparse.kron(sparse.eye(self.N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])

        #TODO Config to adapt based on walls etc.
        Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        A = sparse.vstack([Aeq, Aineq], format='csc')


        # Update input constraints
       
        A = sparse.vstack([Aeq, Aineq], format='csc')
        
        self.l[:self.nx] = -self.x0
        self.u[:self.nx] = -self.x0
        self.prob_x.update(Px = sparse.triu(P).data, Ax=A.data,q=q, l=self.l, u=self.u)
        res = self.prob_x.solve()
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')


    
    def solveMPC_x(self):
        res = self.prob_x.solve()
        # Check solver status
        if res.info.status != 'solved':
            print(res.info.status)
            raise ValueError('OSQP did not solve the problem!')
        self.ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        xTraj = self.I_xy @ res.x[:self.nxc]
        return (res,self.ctrl,xTraj)



    def setupMPC_z(self,egoX,mcN,lambda_JI,rho_JI,x_J):
        # lambda_JI.get(id).shape = N*2, rho_JI.get(id).shape = N*2, x_J.get(id).shape = N*2

        # Intialize matrix variables
        self.mcN = mcN
        P = sparse.eye(0,format='csc')
        q = np.array([])
        A = sparse.eye(0,format='csc')
        A_data = np.array([],dtype='int')
        self.A_rowi = np.array([],dtype='int')
        self.A_coli = np.array([],dtype='int')
        A_rw = 0       
        K = np.zeros(int((len(mcN)*(len(mcN)-1))/2)*self.N)
        for cnt_i, vin_i in enumerate(mcN):
            # Cost function
            P_Rho = sparse.diags(np.hstack(( rho_JI.get(vin_i) + 1e-7)), format='csc')
            P = sparse.block_diag([P , P_Rho],format='csc')
            q_rho = np.multiply(rho_JI.get(vin_i),-x_J.get(vin_i))
            q_lambda = 1/2* lambda_JI.get(vin_i) * 0
            q = np.hstack((q, q_rho + q_lambda))
            # Add linearized collision avoidance between vehicles
            for cnt_j, vin_j in enumerate(mcN):
                if cnt_j <= cnt_i:
                    # Skip values of j which are smaller or equal than i, since i will have had those already
                    continue
                #TODO Correctly assign the data and its rows and column indices
                #TODO Try implementing capsule based collision avoidance
                for i_x in range(self.N):
                    # x vector at linearization point
                    x_bar = np.array([x_J.get(vin_i)[i_x*2],
                    x_J.get(vin_i)[i_x*2+1],
                    x_J.get(vin_j)[i_x*2],
                    x_J.get(vin_j)[i_x*2+1]])
                    # xTA vector at current timestep
                    Ax_i = 2*np.array([x_J.get(vin_i)[i_x*2] - x_J.get(vin_j)[i_x*2],
                    x_J.get(vin_i)[i_x*2+1] - x_J.get(vin_j)[i_x*2+1],
                    x_J.get(vin_j)[i_x*2] - x_J.get(vin_i)[i_x*2],
                    x_J.get(vin_j)[i_x*2+1] - x_J.get(vin_i)[i_x*2+1]])
                    A_data = np.hstack([A_data,Ax_i])
                    K[A_rw] = Ax_i @ x_bar
                    self.A_rowi = np.hstack([self.A_rowi, A_rw, A_rw, A_rw, A_rw]) 
                    A_rw += 1
                    self.A_coli = np.hstack([self.A_coli, cnt_i*self.N*2+i_x*2,cnt_i*self.N*2+i_x*2+1,cnt_j*self.N*2+i_x*2,cnt_j*self.N*2+i_x*2+1]) 
                    # A_it = sparse.hstack( [ np.zeros((1 , cnt_i*self.N*2 + i_x*2)),
                    # x_J.get(vin_i)[i_x*2] - x_J.get(vin_j)[i_x*2],
                    # x_J.get(vin_i)[i_x*2+1] - x_J.get(vin_j)[i_x*2+1], np.zeros((1,self.N*2*len(mcN) - ))]) + sparse.hstack([np.zeros((1, cnt_j*self.N*2 + i_x * 2)) ,
                    # x_J.get(vin_j)[i_x*2] - x_J.get(vin_i)[i_x*2],
                    # x_J.get(vin_j)[i_x*2+1] - x_J.get(vin_i)[i_x*2+1],
                    # np.zeros((1,(len(mcN)-cnt_j)*self.N*2)) ],format='csc')     

        if A_data.size != 0: 
            A = sparse.csc_matrix((A_data,(self.A_rowi,self.A_coli)))
            # lower bound is minimum distance, size depends on mcN len every comb * nxc
            l = np.ones(int((len(mcN)*(len(mcN)-1))/2)*self.N)*(self.d_min*self.d_mult)**2 + K
            # upper bound is inf
            u = np.ones(int((len(mcN)*(len(mcN)-1))/2)*self.N)*np.inf
            # Create an OSQP object
            self.prob_z = osqp.OSQP()
            # Setup workspace
            self.prob_z.setup(P, q, A, l, u, warm_start=True, polish = 1, max_iter = 10000 ,verbose = 0, scaling=20,eps_abs = 1e-8,eps_rel = 1e-4)
        else:
            # Create an OSQP object
            self.prob_z = osqp.OSQP()
            # Setup workspace
            self.prob_z.setup(P, q, warm_start=True, polish = 1 ,verbose = 0)

        # res = self.prob_z.solve()
        # # Check solver status
        # if res.info.status != 'solved':
        #     raise ValueError('OSQP did not solve the problem!')




    def updateMPC_z(self,egoX,mcN,lambda_JI,rho_JI,x_J):
        # lambda_JI.get(id).shape = N*2, rho_JI.get(id).shape = N*2, x_J.get(id).shape = N*2

        # Intialize matrix variables
        self.mcN = mcN        
        P = sparse.eye(0,format='csc')
        q = np.array([])
        A = sparse.eye(0,format='csc')
        A_data = np.array([],dtype='int')
        A_rw = 0       
        K = np.zeros(int((len(mcN)*(len(mcN)-1))/2)*self.N)
        for cnt_i, vin_i in enumerate(mcN):
            # Cost function
            if np.sum(rho_JI.get(vin_i)) == 0:
                rho_JI[vin_i] = rho_JI.get(vin_i) + 1e-7
            P_Rho = sparse.diags(np.hstack(( rho_JI.get(vin_i))), format='csc')
                
            P = sparse.block_diag([P , P_Rho],format='csc')
            q_rho = np.multiply(rho_JI.get(vin_i),-x_J.get(vin_i))
            #TODO debug lambda, setting to zero fixes it, but should not be zero
            q_lambda = 1/2* lambda_JI.get(vin_i) * 0
            q = np.hstack((q, q_rho + q_lambda))
            for cnt_j, vin_j in enumerate(mcN):
                if cnt_j <= cnt_i:
                    # Skip values of j which are smaller or equal than i, since i will have had those already
                    continue
                for i_x in range(self.N):
                    # x vector at linearization point
                    x_bar = np.array([x_J.get(vin_i)[i_x*2],
                    x_J.get(vin_i)[i_x*2+1],
                    x_J.get(vin_j)[i_x*2],
                    x_J.get(vin_j)[i_x*2+1]])
                    # xTA vector at current timestep
                    Ax_i = 2*np.array([x_J.get(vin_i)[i_x*2] - x_J.get(vin_j)[i_x*2],
                    x_J.get(vin_i)[i_x*2+1] - x_J.get(vin_j)[i_x*2+1],
                    x_J.get(vin_j)[i_x*2] - x_J.get(vin_i)[i_x*2],
                    x_J.get(vin_j)[i_x*2+1] - x_J.get(vin_i)[i_x*2+1]])
                    A_data = np.hstack([A_data,Ax_i])
                    K[A_rw] = 1/2*Ax_i @ x_bar
                    A_rw += 1

        if A_data.size != 0: 
            A = sparse.csc_matrix((A_data,(self.A_rowi,self.A_coli)))            
            l = np.ones(int((len(mcN)*(len(mcN)-1))/2)*self.N)*(self.d_min*self.d_mult)**2 + K
            # Update workspace
            self.prob_z.update(Px = sparse.triu(P).data, Ax=A.data,q=q, l=l)
        else:
            # Update workspace
            self.prob_z.update(Px = sparse.triu(P).data, q=q)


        res = self.prob_z.solve()
        #Check solver status
        if res.info.status != 'solved':
            res = self.prob_z.solve()
            if res.info.status != 'solved':
                raise ValueError('OSQP did not solve the problem!')

    
    def solveMPC_z(self):
        res = self.prob_z.solve()
        # Check solver status
        if res.info.status != 'solved':
            res = self.prob_z.solve()
            if res.info.status != 'solved':
                raise ValueError('OSQP did not solve the problem!')
        return (res)

def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print("Toc: start time not set")


if __name__ == '__main__':
    main()        