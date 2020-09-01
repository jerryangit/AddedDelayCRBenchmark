import osqp
import numpy as np
import scipy as sp
from matplotlib import pyplot as plt
from scipy import sparse

# Discrete time model of a quadcopter
dt = 0.1
beta0 = 0
v_ref = 8.33
phi = 0
beta = 0
# Initial and reference states
x0 = np.array([0., 0., 5])
x0_act = x0
xr = np.array([0., 0., v_ref])
Ad = sparse.csc_matrix([
  [1., 0., dt*np.cos(beta0)],
  [0., 1., dt*np.sin(beta0)],
  [0., 0., 1.]
])
Bd = sparse.csc_matrix([
  [0., 0.],
  [0., dt*x0[2]/2.],
  [dt, 0.]])
[nx, nu] = Bd.shape
xDestGlob = np.array([15,1,0])

N = 15    # Prediction horizon
nsim = 150 # N simulate loops

# Initialize for plotting

xData = np.empty((nx,nsim))
xData_act = np.empty((nx,nsim))
uData = np.empty((nu,nsim))


# Constraints
delta_0 = 0 
umin = np.array([-10, np.maximum(-np.pi/4,delta_0-np.pi/16)])
umax = np.array([2.5, np.minimum(np.pi/4,delta_0+np.pi/16)])
xmin = np.array([-np.inf,-np.inf,-5])
xmax = np.array([np.inf,np.inf,12])

# Objective function
Q = sparse.diags([5.,5., 5])
QN = Q*5
R = sparse.diags([0.5, 2.15])
#

# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
# - quadratic objective
P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                       sparse.kron(sparse.eye(N), R)], format='csc')
# - linear objective
# q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
#                np.zeros(N*nu)])

# Calculate reference trajectory
theta = np.arctan2(xDestGlob[1],xDestGlob[0])
vec = np.minimum(((xDestGlob[0] - x0_act[0])**2 + (xDestGlob[1] - x0_act[1])**2)**0.5, dt*v_ref*N)
xDestLoc = np.array([np.cos(phi-theta) * vec , np.sin(phi-theta) * vec , xDestGlob[2]])
x_ref = np.linspace([0,0,x0_act[-1]],xDestLoc,N+1).flatten()
q = np.hstack([np.multiply(np.hstack([np.kron(np.ones(N), Q.diagonal()),QN.diagonal()]),-x_ref),
            np.zeros(N*nu)])
# - linear dynamics
Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
Aeq = sparse.hstack([Ax, Bu])
leq = np.hstack([-x0, np.zeros(N*nx)])
ueq = leq
# - input and state constraints
Aineq = sparse.eye((N+1)*nx + N*nu)
lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
# - OSQP constraints
A = sparse.vstack([Aeq, Aineq], format='csc')
l = np.hstack([leq, lineq])
u = np.hstack([ueq, uineq])

# Create an OSQP object
prob = osqp.OSQP()

# Setup workspace
prob.setup(P, q, A, l, u, warm_start=True)

for i in range(nsim):
    # Solve

    res = prob.solve()

    # Check solver status
    if res.info.status != 'solved':
        raise ValueError('OSQP did not solve the problem!')
      
    # Apply first control input to the plant
    ctrl = res.x[-N*nu:-(N-1)*nu]
    ctrl[1] = np.minimum(np.maximum(ctrl[1],-np.pi/4),np.pi/4)
    xTraj = res.x[:nx*(N+1)]
    xData[:,i] = res.x[0:nx]
    xData_act[:,i] = x0_act
    uData[:,i] = ctrl

    # Update actual states based on bicycle model
    x0_act = np.array([x0_act[0]+dt*x0_act[2]*np.cos(beta+phi), x0_act[1]+dt*x0_act[2]*np.sin(phi+beta), x0_act[2] + dt * ctrl[0] ])
    beta = np.arctan(1/2*np.tan(ctrl[1]))    

    # Update xref based on current location
    phi = phi + ctrl[1]/(1.5)*np.sin(beta)
    vecV2D = np.array([xDestGlob[0] - x0_act[0], xDestGlob[1] - x0_act[1]])
    theta = np.arctan2(vecV2D[1],vecV2D[0])
    vec = np.minimum(((xDestGlob[0] - x0_act[0])**2 + (xDestGlob[1] - x0_act[1])**2)**0.5, dt*v_ref*N)
    xDestLoc = np.array([np.cos(theta-phi) * vec , np.sin(theta-phi) * vec , xDestGlob[2]])
    x_ref = np.linspace([0,0,x0_act[-1]],xDestLoc,N+1).flatten()
    q = np.hstack([np.multiply(np.hstack([np.kron(np.ones(N), Q.diagonal()),QN.diagonal()]),-x_ref),
               np.zeros(N*nu)])


    
    # Update dynamics
    Ad = sparse.csc_matrix([
      [1., 0., dt*np.cos(beta)],
      [0., 1., dt*np.sin(beta)],
      [0., 0., 1.]
    ])
    Bd = sparse.csc_matrix([
      [0., 0.],
      [0., dt*x0_act[2]/2.],
      [dt, 0.]])
    Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
    Aeq = sparse.hstack([Ax, Bu])
    A = sparse.vstack([Aeq, Aineq], format='csc')
    # Update input constraints
    delta_0 = ctrl[1]
    umin = np.array([-10, np.maximum(-np.pi/4,delta_0-np.pi/16)])
    umax = np.array([2.5, np.minimum(np.pi/4,delta_0+np.pi/16)])
    lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
    uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])


    # Update initial state
    l[:nx] = np.array([0,0,-x0_act[-1]])
    u[:nx] = np.array([0,0,-x0_act[-1]])
    
    prob.update(Ax=A.data, q=q, l=l, u=u)

plt.rc('text', usetex=True)
# xPlot, = plt.plot(xData[0,:])
# yPlot, = plt.plot(xData[1,:])
# vPlot, = plt.plot(xData[2,:])
xPlot_act, = plt.plot(xData_act[0,:])
yPlot_act, = plt.plot(xData_act[1,:])
vPlot_act, = plt.plot(xData_act[2,:])
aPlot, = plt.plot(uData[0,:])
deltaPlot, = plt.plot(uData[1,:])
# plt.legend((xPlot, yPlot, vPlot,aPlot,deltaPlot, xPlot_act, yPlot_act, vPlot_act), ('x', 'y', 'v','a','$\delta$','xGlob','yGlob','vGlob'))
plt.legend((aPlot,deltaPlot, xPlot_act, yPlot_act, vPlot_act), ('a','$\delta$','xGlob','yGlob','vGlob'))

plt.show()
