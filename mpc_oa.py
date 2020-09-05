import osqp
import numpy as np
import scipy as sp
import copy
import sys
import matplotlib.pyplot as plt
from scipy import sparse

def main():
    dt = 0.1
    d_min = 3
    d_mult = 1.75
    rho_base = 1
    phi_a = 6
    mu_0 = 0.25
    N = 10                     # Prediction horizon
    mcN_Dist = 25              # Distance at vehicle is added to mcN
    mpc = oa_mpc(dt,N,d_min,d_mult)
    egoX = egoXDummy()
    xDestLoc = np.array([10,1,egoX.velRef])
    x_ref =  np.linspace([0,0,egoX.velRef],xDestLoc,N+1).flatten()
    mcN = set([1,2])
    z_JI = {}
    x_J = {}
    lambda_JI = {}
    rho_JI = {}
    lambda_JI[1] = np.ones(3*N+3)
    lambda_JI[2] = np.ones(3*N+3)
    rho_JI[1] = np.ones(3*N+3)
    rho_JI[2] = np.ones(3*N+3)
    z_JI[1] = np.linspace([0,0,0],xDestLoc + np.array((-4,1,0)),N+1).flatten()
    z_JI[2] = np.linspace([0,0,0],xDestLoc + np.array((-4,1,0)),N+1).flatten()
    mpc.setupMPC_x(egoX)    
    mpc.updateMPC_x(egoX,x_ref,z_JI,lambda_JI,rho_JI,mcN)
    (res,ctrl,xTraj) = mpc.solveMPC_x()
    # plt.plot(xTraj[::2])
    # plt.plot(xTraj[1::2])    
    # plt.plot(res.x[2:33:3])    
    # plt.legend(('x','y','v'))
    # plt.show()
    x_J[1] = np.linspace([0,0,0],xDestLoc, N+1).flatten()
    x_J[2] = np.linspace([0,0,0],xDestLoc, N+1).flatten()    
    mpc.setupMPC_z(egoX,mcN,lambda_JI, rho_JI, x_J)    
    mpc.updateMPC_z(egoX,x_ref,z_JI,lambda_JI,rho_JI,mcN)
    (res,ctrl,xTraj) = mpc.solveMPC_z()


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
        # Not configurable
        self.nx = 3                     # Number of states
        self.nu = 2                     # Number of inputs
        self.nxc = self.nx*(self.N+1)   # Number of states concatenated
        self.nuc = self.nu*self.N       # Number of inputs concatenated
        self.nucZ = np.zeros(self.nuc)     # Empty array of zeros of shape (nuc,)
        self.delta = 1e-5                  # Initialize delta
        self.beta = 1e-5                   # Initialize beta
        self.ddeltaMax = np.pi/4*self.dt   # Maximum steering angle change per step
        self.I_xy = sparse.kron(np.eye(self.N+1),np.array(([1., 0., 0.], [0., 1.,0.]))) # Indicator matrix to get a vector with only XY coordinates shape of 2*(N+1)
        self.ctrl = (1e-5,1e-5)

    def setupMPC_x(self,egoX):
        self.v_ref = egoX.velRef
        self.x0 = np.array((0.,0.,egoX.velNorm))
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
            [0., 0.],
            [0., self.dt*self.x0[2]/2.],
            [self.dt, 0.]
        ])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(self.N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        self.leq = np.hstack([-self.x0, np.zeros(self.N*self.nx)])
        self.ueq = self.leq

        # Input constraints 
        umin = np.array([-10, np.maximum(-np.pi/4,self.delta-self.ddeltaMax)])
        umax = np.array([2.5, np.minimum(np.pi/4,self.delta+self.ddeltaMax)])
        # Ineq constraints, collision avoidance
        xmin = np.array([-np.inf,-np.inf,-3])
        xmax = np.array([np.inf,np.inf,12])
        Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        lineq = np.hstack([np.kron(np.ones(self.N+1), xmin), np.kron(np.ones(self.N), umin)])
        uineq = np.hstack([np.kron(np.ones(self.N+1), xmax), np.kron(np.ones(self.N), umax)])

        # Cost function
        Q = sparse.diags([1., 10., 10.])
        QN = Q*5
        R = sparse.diags([1., 10.])
        x_ref = np.linspace([0,0,self.x0[2]],[1,1,self.v_ref],self.N+1).flatten()


        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        self.P_Q = sparse.block_diag([sparse.kron(sparse.eye(self.N), Q), QN, sparse.kron(sparse.eye(self.N), R,format='csc')], format='csc') 
        P = self.P_Q        
        # - linear objective
        self.lambda_ref = np.hstack([np.kron(np.ones(self.N), Q.diagonal()),QN.diagonal()])
        q = np.hstack([np.multiply(self.lambda_ref,-x_ref), self.nucZ])         

        A = sparse.vstack([Aeq, Aineq], format='csc')
        l = np.hstack([self.leq, lineq])
        u = np.hstack([self.ueq, uineq])
        P,q 
        # Create an OSQP object
        self.prob_x = osqp.OSQP()
        # Setup workspace
        self.prob_x.setup(P, q, A, l, u, warm_start=True, polish = 1)


    def updateMPC_x(self,egoX,x_ref,z_JI,lambda_JI,rho_JI,mcN):
        self.x0 = np.array([0,0,egoX.velNorm])
        self.beta = np.arctan(1/2*np.tan(self.ctrl[1]))  
        # Update Q and q based on OA-ADMM rules # q = lambda.*(-reference)
        q = np.hstack([np.multiply(self.lambda_ref,-x_ref), self.nucZ])
        P = self.P_Q  
        for vin in mcN:
            q_rho = np.hstack([np.multiply(rho_JI.get(vin),-z_JI.get(vin)), self.nucZ])
            q_lambda = np.hstack((1/2*lambda_JI.get(vin),self.nucZ))
            q = q + q_rho + q_lambda
            P_Rho = sparse.diags(np.hstack((rho_JI.get(vin),np.zeros(self.nuc))), format='csc')
            P = P + P_Rho
        Ad = sparse.csc_matrix([
        [1., 0., self.dt*np.cos(self.beta)],
        [0., 1., self.dt*np.sin(self.beta)],
        [0., 0., 1.]
        ])
        Bd = sparse.csc_matrix([
        [0., 0.],
        [0., self.dt*self.x0[2]/2.],
        [self.dt, 0.]])
        Ax = sparse.kron(sparse.eye(self.N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])

        #TODO Config to adapt based on walls etc.
        Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        A = sparse.vstack([Aeq, Aineq], format='csc')


        # Update input constraints
        xmin = np.array([-np.inf,-np.inf,-3])
        xmax = np.array([np.inf,np.inf,12])
        umin = np.array([-10, np.maximum(-np.pi/4,self.delta-self.ddeltaMax)])
        umax = np.array([2.5, np.minimum(np.pi/4,self.delta+self.ddeltaMax)])
        lineq = np.hstack([np.kron(np.ones(self.N+1), xmin), np.kron(np.ones(self.N), umin)])
        uineq = np.hstack([np.kron(np.ones(self.N+1), xmax), np.kron(np.ones(self.N), umax)])
        
        A = sparse.vstack([Aeq, Aineq], format='csc')

        l = np.hstack([self.leq, lineq])
        u = np.hstack([self.ueq, uineq])
        
        l[:self.nx] = -self.x0
        u[:self.nx] = -self.x0
        self.prob_x.update(Px = sparse.triu(P).data, Ax=A.data,q=q, l=l, u=u)

    
    def solveMPC_x(self):
        res = self.prob_x.solve()
        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')
        self.ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        xTraj = self.I_xy*res.x[:self.nxc]
        return (res,self.ctrl,xTraj)



    def setupMPC_z(self,egoX,mcN,lambda_JI,rho_JI,x_J):
        P = sparse.csc_matrix([])
        q = np.array([])
        for vin in mcN:
            # Cost function
            P_Rho = sparse.diags(np.hstack((rho_JI.get(vin))), format='csc')
            P = sparse.block_diag([P , P_Rho],format='csc')
            q_rho = np.multiply(rho_JI.get(vin),-x_J.get(vin))
            q_lambda = 1/2*lambda_JI.get(vin)
            q = np.hstack((q, q_rho + q_lambda))

            # lower bound is minimum distance, size depends on mcN len every comb * nxc
            l = np.ones(len(mcN))*self.d_min*self.d_mult
            # upper bound is inf
            u = np.ones(len(mcN))*np.inf
            # Add linearized collision avoidance between vehicles
            for vin in mcN:
                A = sparse.diags(np.ones(1))

           

        # Create an OSQP object
        tic()
        self.prob_z = osqp.OSQP()
        toc()
        # Setup workspace
        tic()
        self.prob_z.setup(P, q, A, l, u, warm_start=True, polish = 1)
        toc()

        res = self.prob_z.solve()
        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        for vin in mcN:
            self.z_IJ[vin] = res.x[1:20]



    def updateMPC_z(self,egoX,x_ref,z_JI,lambda_JI,rho_JI,mcN):
        # Update Q and q based on OA-ADMM rules # q = lambda.*(-reference)
        q = np.hstack([np.multiply(self.lambda_ref,-x_ref), self.nucZ])
        P = self.P_Q  
        for vin in mcN:
            q_rho = np.hstack([np.multiply(rho_JI.get(vin),-z_JI.get(vin)), self.nucZ])
            q_lambda = np.hstack((1/2*lambda_JI.get(vin),self.nucZ))
            q = q + q_rho + q_lambda
            P_Rho = sparse.diags(np.hstack((rho_JI.get(vin),np.zeros(self.nuc))), format='csc')
            P = P + P_Rho

        Ad = sparse.csc_matrix([
        [1., 0., self.dt*np.cos(self.beta)],
        [0., 1., self.dt*np.sin(self.beta)],
        [0., 0., 1.]
        ])
        Bd = sparse.csc_matrix([
        [0., 0.],
        [0., self.dt*self.x0[2]/2.],
        [self.dt, 0.]])
        Ax = sparse.kron(sparse.eye(self.N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])

        #TODO Config to adapt based on walls etc.
        Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        A = sparse.vstack([Aeq, Aineq], format='csc')


        # Update input constraints
        xmin = np.array([-np.inf,-np.inf,-3])
        xmax = np.array([np.inf,np.inf,12])
        umin = np.array([-10, np.maximum(-np.pi/4,self.delta-self.ddeltaMax)])
        umax = np.array([2.5, np.minimum(np.pi/4,self.delta+self.ddeltaMax)])
        lineq = np.hstack([np.kron(np.ones(self.N+1), xmin), np.kron(np.ones(self.N), umin)])
        uineq = np.hstack([np.kron(np.ones(self.N+1), xmax), np.kron(np.ones(self.N), umax)])
        
        A = sparse.vstack([Aeq, Aineq], format='csc')

        l = np.hstack([self.leq, lineq])
        u = np.hstack([self.ueq, uineq])
        
        l[:self.nx] = -self.x0
        u[:self.nx] = -self.x0
        self.prob_x.update(Px = sparse.triu(P).data, Ax=A.data,q=q, l=l, u=u)

    
    def solveMPC_z(self):
        res = self.prob_x.solve()
        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')
        self.ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        xTraj = self.I_xy*res.x[:self.nxc]
        return (res,self.ctrl,xTraj)

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