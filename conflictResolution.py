#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import conflictDetection as cd
import deadlockDetection as dd
import mpc_oa as mpc
import glob
import os
import sys
import datetime
import osqp
import scipy as sp
import copy
from scipy import sparse

# try:
#     sys.path.append(glob.glob('../carla/dist/carla-*%d.5-%s.egg' % (
#         sys.version_info.major,
#         # sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass

import carla
import numpy as np

class msg:
    def __init__(self,idSend,mtype="N/A",content={}):
        self.mtype = mtype
        self.idSend = idSend
        self.content = content
    
class priorityPolicy:
    def __init__(self,policy):
        self.policy = policy

    def order(self,para):
        cases = {
            "FCFS": self.FCFS,
            "TimeSpent": self.TimeSpent,
            "PriorityScore": self.PriorityScore
        }
        fnc = cases.get(self.policy)
        #* 1 = ego holds priorty, 0 = ego has to yield
        return fnc(para)
        
    def PriorityScore(self,actorList):
        actorList.sort(key = lambda x: (round(x.cr.priorityScore,1),x.VIN))
        return actorList

    def FCFS(self,actorList):
        actorList.sort(key = lambda x: (round(x.cr.cd.arrivalTime,1),x.VIN))
        return actorList

    def TimeSpent(self,actorList):
        actorList.sort(key = lambda x: (round(x.cr.cd.intersectionTime,1),x.VIN))
        return actorList

        
class conflictResolution:
    def __init__(self,method,para=[]):
        self.method = method
        self.obj = self.switchCreate(method,para)

    def switchCreate(self,arg,para):
        cases = {
            "TEP": TEP,
            "MPIP": MPIP,
            "AMPIP": AMPIP,
            "DCR":DCR,
            "OAADMM":OAADMM,
        }
        fnc = cases.get(arg)
        return fnc(*para)

class TEP:
    #! Fix deadlocks by changing logic, vehicles are removed from waitlist they have lower Priority 
    def __init__(self,errMargin,policy):
        self.errMargin = errMargin
        self.wait ={}
        self.pp = priorityPolicy(policy)

    def resolve(self,egoX,worldX):
        # [self.ttArrival, self.ttExit] = self.cd.predictTimes(egoX,worldX)
        if len(worldX.msg.inbox) == 0:
            return 0
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.mtype == "STOP":
                actorX = worldX.actorDict.dict.get(msg.idSend)
                [bool,TIC] = self.cd.detect(egoX,worldX,msg)
                if bool:
                    pOrder = self.pp.order([egoX,actorX])
                    if pOrder[0].id == egoX.id:
                        if msg.idSend in self.wait:
                            del self.wait[msg.idSend]
                    else:
                        if msg.idSend not in self.wait:
                            self.wait[msg.idSend] = TIC
                else: 
                    continue
            elif msg.mtype == "CLEAR":
                if msg.idSend in self.wait:
                    del self.wait[msg.idSend]
        # Updated at the end to ensure all vehicles utilize the information from the same time step
        egoX.cr.cd.predictTimes(egoX,worldX)

    def outbox(self,actorX):
        if actorX.state == "ENTER" or actorX.state == "CROSS":
            msg_obj = msg(actorX.id,"STOP",{"timeSlot":[self.cd.arrivalTime,self.cd.exitTime],"TCL":self.cd.TCL})
            return msg_obj
        elif actorX.state == "EXIT":
            msg_obj = msg(actorX.id,"CLEAR")
            return msg_obj

    def setup(self,egoX,worldX):
        # self.cd = cd.conflictDetection("timeSlot",[self.errMargin]).obj
        # TODO Grid TCL is more like MP-IP fix later
        self.cd = cd.conflictDetection("gridCell",[worldX.inter_location,4,18,self.errMargin]).obj
        self.cd.setup(egoX,worldX)

class MPIP:
    def __init__(self,errMargin,policy):
        self.errMargin = errMargin
        self.wait ={}
        self.pp = priorityPolicy(policy)

    def resolve(self,egoX,worldX):
        self.cd.updateTCL(egoX.sTraversed)
        if len(worldX.msg.inbox) == 0:
            return 0
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.mtype == "ENTER" or msg.mtype == "CROSS":
                actorX = worldX.actorDict.dict.get(msg.idSend)
                TIC = self.cd.detect(egoX,worldX,msg)
                [sptBool,TIC] = self.cd.detect(egoX,worldX,msg)
                if sptBool == 1:
                    pOrder = self.pp.order([egoX,actorX])
                    if pOrder[0].id == egoX.id:
                        if msg.idSend in self.wait:
                            del self.wait[msg.idSend]
                    else:
                        if msg.idSend not in self.wait:
                            self.wait[msg.idSend] = TIC
                else: 
                    # If there is no conflict remove it from the wait list
                    if msg.idSend in self.wait:
                        del self.wait[msg.idSend]
            elif msg.mtype == "EXIT":
                if msg.idSend in self.wait:
                    del self.wait[msg.idSend]        
        # Updated at the end to ensure all vehicles utilize the information from the same time step
        egoX.cr.cd.predictTimes(egoX,worldX)
        self.cd.updateTCL(egoX.sTraversed)

    def outbox(self,actorX):
        if actorX.state == "ENTER":
            msg_obj = msg(actorX.id,"ENTER",{"timeSlot":(self.cd.arrivalTime,self.cd.exitTime),"TCL":self.cd.TCL})
            return msg_obj
        if actorX.state == "CROSS":
            msg_obj = msg(actorX.id,"CROSS",{"timeSlot":(self.cd.arrivalTime,self.cd.exitTime),"TCL":self.cd.TCL})
            return msg_obj
        elif actorX.state == "EXIT":
            msg_obj = msg(actorX.id,"EXIT")
            return msg_obj

    def setup(self,egoX,worldX):
        self.cd = cd.conflictDetection("gridCell",[worldX.inter_location,4,18,self.errMargin]).obj
        self.cd.setup(egoX,worldX)

class AMPIP:
    def __init__(self,errMargin,policy):
        self.errMargin = errMargin
        self.wait ={}
        self.pp = priorityPolicy(policy)

    def resolve(self,egoX,worldX):
        if len(worldX.msg.inbox) == 0:
            return 0
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.mtype == "ENTER" or msg.mtype == "CROSS":
                actorX = worldX.actorDict.dict.get(msg.idSend)
                #!Required modification, trajectory intersecting cell list needed in order to guarantee safe crossing
                [sptBool,TICL] = self.cd.detect(egoX,worldX,msg,1)
                if sptBool == 1:
                    pOrder = self.pp.order([egoX,actorX])
                    if pOrder[0].id == egoX.id:
                        if msg.idSend in self.wait:
                            del self.wait[msg.idSend]
                    else:
                        if msg.idSend not in self.wait:
                            AMP_yield = 0
                            for cell in TICL:
                                if self.cd.traj.get(cell)[1] + self.errMargin > msg.content.get("traj").get(cell)[0]:
                                    self.wait[msg.idSend] = cell
                                    AMP_yield = 1
                                    break
                            if AMP_yield == 0 :
                                #! Added to avoid multi vehicle AMP deadlocks.
                                if len(self.wait) <= 1:
                                    if msg.idSend in self.wait:
                                        del self.wait[msg.idSend]
                else: 
                    if msg.idSend in self.wait:
                        # if egoX._spwnNr == 68:
                        #     print(egoX.id," No longer has a conflict with ", msg.idSend)
                        del self.wait[msg.idSend]
            elif msg.mtype == "EXIT":
                if msg.idSend in self.wait:
                    del self.wait[msg.idSend]
        # Updated at the end to ensure all vehicles utilize the information from the same time step
        egoX.cr.cd.predictTimes(egoX,worldX)
        self.cd.updateTCL(egoX.sTraversed)


    def outbox(self,egoX):
        if egoX.state == "ENTER":
            msg_obj = msg(egoX.id,"ENTER",{"timeSlot":(self.cd.arrivalTime,self.cd.exitTime),"TCL":self.cd.TCL,"traj":self.cd.traj})
            return msg_obj
        if egoX.state == "CROSS":
            msg_obj = msg(egoX.id,"CROSS",{"timeSlot":(self.cd.arrivalTime,self.cd.exitTime),"TCL":self.cd.TCL,"traj":self.cd.traj})
            return msg_obj
        elif egoX.state == "EXIT":
            msg_obj = msg(egoX.id,"EXIT")
            return msg_obj

    def setup(self,egoX,worldX):
        self.cd = cd.conflictDetection("gridCell",[worldX.inter_location,4,18,self.errMargin]).obj
        self.cd.setup(egoX,worldX)

class DCR:
    def __init__(self,errMargin,policy):
        self.errMargin = errMargin
        self.wait ={} # Wait dictionary
        self.tmp = {} # Temporal advantage
        self.pp = priorityPolicy(policy)
        self.dd = dd.deadlockDetection("DCRgraph").obj

    def resolve(self,egoX,worldX):
        if len(worldX.msg.inbox) == 0:
            return 0
        self.wait = {} # Clear yield list every time
        self.tmp = {} # Clear tmp every time?
        self.updateData(egoX,worldX)
        if egoX.state == "IL":
            if egoX.info.get("idFront") != None:
                TICL = egoX.cr.cd.TICL(worldX.actorDict.dict.get(egoX.info.get("idFront")).cr.cd.TCL)
                traj = worldX.actorDict.dict.get(egoX.info.get("idFront")).cr.cd.traj
                self.wait[egoX.info.get("idFront")] = ["Queue",TICL,traj]
        elif egoX.state == "OL":
            return 0 

        for msg in worldX.msg.inbox[egoX.id]:
            if msg.content.get("state") in ["NAN","IL"]:
                continue
            elif msg.idSend == egoX.info.get("idFront") and msg.content.get("state") == "I":
                egoX.discreteState("FIL")
                # egoX.infoSet("idFront",None)
            elif msg.content.get("state") == "OL":
                if msg.idSend in self.tmp:
                    del self.tmp[msg.idSend]
                if msg.idSend in self.wait:
                    del self.wait[msg.idSend]
                continue
            (sptBool,TICL) = self.cd.sptDetect(egoX,worldX,msg)
            tmpBoolEgo = self.tempAdvantage(TICL,egoX.cr.cd.traj,msg.content.get("traj"),egoX.state,msg.content.get("state"))
            tmpBoolAct = self.tempAdvantage(TICL,msg.content.get("traj"),egoX.cr.cd.traj,msg.content.get("state"),egoX.state)
            # If Ego has a spatial conflict with msg sender
            if sptBool == 1:
                # If msg sender has temporal advantage over Ego
                if tmpBoolAct == 1:
                    self.tmp[msg.idSend] = msg.content.get("state")
                    actorX = worldX.actorDict.dict.get(msg.idSend)
                    pOrder = self.pp.order([egoX,actorX])
                    # If there is no Tie(Ego,Actor) or Actor has priority over Ego
                    if self.dd.yieldSearch(self.tmp,egoX.id,msg.idSend,egoX.state,msg.content.get("state")) == 0 or pOrder[0].id == msg.idSend:
                        self.wait[msg.idSend] = [msg.content.get("state"),TICL,msg.content.get("traj")]
                # If Ego has temporal advantage over msg sender
                if tmpBoolEgo == 1:  
                    actorX = worldX.actorDict.dict.get(msg.idSend)
                    pOrder = self.pp.order([egoX,actorX])
                    # If there is a Tie(Actor,Ego) and Actor has priority over Ego
                    if self.dd.yieldSearch(msg.content.get("tmp"),msg.idSend,egoX.id,msg.content.get("state"),egoX.state) == 1 and pOrder[0].id == msg.idSend:
                        self.wait[msg.idSend] = [msg.content.get("state"),TICL,msg.content.get("traj")]

        for idYield in self.wait:
            for cell in self.wait.get(idYield)[1]:
                # Prediction of time actor leaves cell + errMargin for robustness               
                if self.wait.get(idYield)[0] == "Queue":
                    # Extra self.errMargin to avoid collision in queue
                    tOutAct = self.wait.get(idYield)[2].get(cell)[1] + self.errMargin * 4
                else:
                    tOutAct = self.wait.get(idYield)[2].get(cell)[1] + self.errMargin * 2
                if self.cd.traj[cell][1] < self.cd.traj[cell][0]:
                    print("tIn<tOut")
                # Current time for ego to enter cell
                tinEgo = self.cd.traj[cell][0]
                # if the yielding changes the Ego vehicles times
                if tOutAct > tinEgo:
                    delay = tOutAct - tinEgo
                    # If the vehicle is waiting for a vehicle in front of it, delay all the cells.
                    if self.wait.get(idYield)[0] == "Queue":
                        for cell in self.cd.traj:
                            self.cd.traj[cell] = (self.cd.traj[cell][0] + delay, self.cd.traj[cell][1] + delay)
                    # if self.cd.TCL.index(cell) < len(self.cd.TCL) - 1:
                    # Amount of cells that need to be updated with the new delay
                    cellIndex = self.cd.TCL.index(cell)
                    delayedCells = len(self.cd.TCL) - cellIndex
                    for i in range(delayedCells):
                        cell = self.cd.TCL[i+cellIndex]
                        self.cd.traj[cell] = (self.cd.traj[cell][0] + delay, self.cd.traj[cell][1] + delay)
                if self.cd.traj[cell][1] < self.cd.traj[cell][0]:
                    print("tIn<tOut")

    def tempAdvantage(self,TICL,egoTraj,actorTraj,egoState,actState):
        # If Ego is crossing and Actor is first in lane
        if egoState == "I" and actState == "FIL":
            for conCell in TICL:
                egoT = egoTraj.get(conCell)
                actorT = actorTraj.get(conCell)
                # If Ego enters a conflict zone before Actor leaves => Ego >TempAdv Actor 
                if egoT[0] < actorT[1]:
                    return 1
        # If Ego is first in line and Actor is in the intersection
        elif egoState == "FIL" and actState == "I":
            for conCell in TICL:
                egoT = egoTraj.get(conCell)
                actorT = actorTraj.get(conCell)
                # If Ego leaves any conflict zone after Actor enters => Actor >TempAdv Ego 
                if egoT[1] > actorT[0]:
                    return 0
            # If Ego leaves all conflict zones before Actor enters => Ego >TempAdv Actor 
            return 1
        # If Ego and Actor have the same state, being either FIL or I
        elif egoState == actState and ( egoState == "FIL" or egoState == "I") :
            for conCell in TICL:
                egoT = egoTraj.get(conCell)
                actorT = actorTraj.get(conCell)
                # If Ego enters any conflict zone before Actor enters => Ego >TempAdv Actor 
                if egoT[0] < actorT[0]:
                    return 1
        return 0

    def updateData(self,egoX,worldX):
        #  Loop needed to process msgs in a useable format
        for msg in worldX.msg.inbox[egoX.id]:
            # Complete graph using received msgs
            self.dd.addEdgeList(msg.idSend,msg.content.get("tmp"))
            self.dd.setState(msg.idSend,msg.content.get("state"))      
        
    def outbox(self,egoX):
        content = {}
        content["state"] = egoX.state
        content["TCL"] = self.cd.TCL
        content["traj"] = self.cd.traj
        content["tmp"] = self.tmp
        content["wp"] = egoX.waypoint
        content["spwnid"] = egoX.spwnid
        #content[wait] = self.wait
        msg_obj = msg(egoX.id,"COM",content)
        return msg_obj

    def setPriority(self,score):
        self.priorityScore = score


    def setup(self,egoX,worldX):
        self.cd = cd.conflictDetection("conflictZones",[worldX.inter_location,2,8,self.errMargin]).obj
        self.cd.setup(egoX,worldX)
        self.setPriority(0)

class OAADMM:
    def __init__(self,errMargin,policy):
        self.errMargin = errMargin
        self.pp = priorityPolicy(policy)
        self.dd = dd.deadlockDetection("DCRgraph").obj
        self.mcN = []
        self.x_i = np.array([])
        self.x_J = {}
        self.x_J0 = {}
        self.theta_J = {}
        self.z_IJ = {}
        self.z_JI = {}
        self.lambda_JI = {}
        self.lambda_IJ = {}
        self.rho_JI = {}
        self.rho_IJ = {}
        ## OA-ADMM Parameter
        self.dt = 0.1
        self.d_mult = 1.75
        self.rho_base = 1
        self.phi_a = 6
        self.mu_0 = 0.25
        self.N = 10                     # Prediction horizon
        self.mcN_Dist = 25              # Distance at vehicle is added to mcN
        self.mpc = mpc.oa_mpc(self.dt,self.N,self.d_mult,self.N)

        self.I_xy = sparse.kron(np.hstack((np.zeros((self.N,1)),np.eye(self.N))),np.array(([1., 0., 0.], [0., 1.,0.]))) # Indicator matrix to get a vector with only XY coordinates shape of 2*N        
        self.I_xyv = sparse.kron(np.vstack([np.zeros((1,self.N)),np.eye(self.N)]),np.array(([1., 0.], [0., 1.],[0.,0.]))) # Indicator matrix to get a vector with only XY coordinates shape of 2*N        
    
    def gen_x_ref(self):
        self.x_ref = np.array()


    def xUpdate(self,egoX,worldX):
        mcN_copy = copy.copy(self.mcN)
        # Construct and update Neighbor set
        for msg in worldX.msg.inbox[egoX.id]:
            if egoX.id == msg.idSend:
                continue
            elif msg.idSend in self.mcN:
                if egoX.location.distance(msg.content.get("loc")) > self.mcN_Dist:
                    self.mcN.remove(msg.idSend)
                else:
                    continue
            elif 0 < egoX.location.distance(msg.content.get("loc")) <= self.mcN_Dist:
                self.mcN.append(msg.idSend)
        if mcN_copy == self.mcN:
            self.mcN_change = 0 
        else:
            self.mcN_change = 1
        

        # Receive z_JI, lambda_JI from neighbor set
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.idSend in self.mcN:
                self.theta_J[msg.idSend] = msg.content.get("theta")
                # If z_JI doesn't work like this add x0 to first state
                self.z_JI[msg.idSend] = self.I_xyv @ msg.content.get("z_IJ").get(egoX.id)
                self.lambda_JI[msg.idSend] = self.I_xyv @ msg.content.get("lambda_IJ").get(egoX.id)
        #TODO Generate reference x based on path
        self.gen_x_ref()
        # Update MPC data
        self.mpc.updateMPC_x(egoX,self.x_ref,self.z_JI,self.lambda_JI,self.rho_JI,self.mcN)
        # Solve problem
        (res,ctrl,self.x_i) = self.mpc.solveMPC_x()
        # Save current theta
        self.theta = ((np.pi*egoX.rotation.yaw)/180)%2*np.pi


    def zUpdate(self,egoX,worldX):
        # Receive z_JI, lambda_JI from neighbor set
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.idSend == egoX.id:
                print('ERROR!, if never error remove this code')
                continue
            if msg.idSend in self.mcN:
                self.x_J0[msg.idSend] = msg.content.get("x_i")
                # Convert x_i into our ego coordinate system 
                Tps_ji = rMatrix(self.theta_J) @ np.array([egoX.location.x-worldX.actorDict.get(msg.idSend).location.x,egoX.location.y - worldX.actorDict.get(msg.idSend).location.y])
                self.x_J[msg.idSend] = np.kron(np.ones((self.N,1)),rMatrixAB(self.theta_J[msg.idSend],self.theta)) @ (msg.content.get("x_i") - np.kron(np.ones((self.N,1)),Tps_ji)) 
                self.lambda_JI[msg.idSend] = msg.content.get("lambda_IJ").get(egoX.id)


        if self.mcN_change == 1:
            # z-Update optimization
            self.mpc.setupMPC_z(egoX,self.mcN,self.lambda_JI,self.rho_JI,self.x_J)
        else:
            # Update MPC data
            self.mpc.updateMPC_z(egoX,self.mcN,self.lambda_JI,self.rho_JI,self.x_J)

        # Solve problem
        (res) = self.mpc.solveMPC_z()

        for cnt_i, vin_i in enumerate(self.mcN):
            if vin_i == egoX.id:
                self.z_IJ[vin_i] = res.x[cnt_i*self.N*2:(cnt_i+1)*self.N*2]
            else:
                z_ij_loc = res.x[cnt_i*self.N*2:(cnt_i+1)*self.N*2]
                Tps_ij = rMatrix(self.theta) @ np.array([worldX.actorDict.get(vin_i).location.x-egoX.location.x,worldX.actorDict.get(vin_i).location.y-egoX.location.y])
                self.z_IJ[vin_i] = np.kron(np.ones((self.N,1)),rMatrixAB(self.theta,self.theta_J[vin_i] )) @ (z_ij_loc - np.kron(np.ones((self.N,1)),Tps_ij))

        # Update lambda 
        for vin_i in range(self.mcN):
            self.lambda_IJ[vin_i] = self.mu_0*self.lambda_IJ[vin_i] + self.rho_IJ[vin_i] @ (self.x_J0[vin_i]- self.z_IJ[vin_i])
        
        # Update rho
        for vin_i in range(self.mcN):
            #TODO figure out how rho works, and how to do lambda_ij vs lambda_ji
            self.rho_IJ = []
        
        

    def updateData(self,egoX,worldX):
        #  Loop needed to process msgs in a useable format
        # for msg in worldX.msg.inbox[egoX.id]:
        self.x0 = np.zeros(0,0,egoX.velNorm)
        pass

    def outbox(self,egoX,var):
        content = {} 
        # OA-ADMM Specific
        content["loc"] = egoX.location
        if var == "xUpdate":
            content["x_i"] = self.x_i
            content["theta"] = self.theta
        elif var == "zUpdate":
            #TODO make sure correct info is communicated
            content["z_IJ"] = self.z_IJ
            content["lambda_IJ"] = self.lambda_IJ
        msg_obj = msg(egoX.id,"COM",content)
        return msg_obj

    def setPriority(self,score):
        self.priorityScore = score

    def setup(self,egoX,worldX):
        self.mcN.append(egoX.id)
        self.mpc.setupMPC_x(egoX)
        self.setPriority(0)        


def rMatrix(theta):
    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    return R
def rMatrixAB(thetaA,thetaB):
    R = np.array([[np.cos(thetaA-thetaB),-np.sin(thetaA-thetaB)],[np.sin(thetaA-thetaB),np.cos(thetaA-thetaB)]])
    return R
def tMatrix(x1,x2):
    T = x2-x1
    return T
def transform(theta,xy1,xy2,vec):
    R = rMatrix(theta)
    T = tMatrix(xy1,xy2)
    vec = R @ (vec-T)
    return vec
