#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import conflictDetection as cd
import deadlockDetection as dd
import glob
import os
import sys
import datetime

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.5-%s.egg' % (
        sys.version_info.major,
        # sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

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
            "TEP_fix": TEP_fix,
            "MPIP": MPIP,
            "AMPIP": AMPIP,
            "DCR":DCR,
        }
        fnc = cases.get(arg)
        return fnc(*para)

class TEP:
    #! TEP has deadlock if arrivalTime is a changing estimate, design is flawed, cannot avoid deadlocks!!!
    def __init__(self,err,policy):
        self.err = err
        self.wait =[]
        self.pp = priorityPolicy(policy)

    def resolve(self,egoX,worldX):
        # [self.ttArrival, self.ttExit] = self.cd.predictTimes(egoX,worldX)
        if len(worldX.msg.inbox) == 0:
            return 0
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.mtype == "STOP" and msg.idSend not in self.wait:
                actorX = worldX.actorDict.dict.get(msg.idSend)
                [bool,TIC] = self.cd.detect(egoX,worldX,msg)
                if bool:
                    pOrder = self.pp.order([egoX,actorX])
                    if pOrder[0].id == egoX.id:
                        continue
                    else:
                        self.wait.append(msg.idSend)
                else: 
                    continue
            elif msg.mtype == "CLEAR":
                if msg.idSend in self.wait:
                    self.wait.remove(msg.idSend)

    def outbox(self,actorX):
        if actorX.state == "ENTER" or actorX.state == "CROSS":
            msg_obj = msg(actorX.id,"STOP",{"timeSlot":[self.cd.arrivalTime,self.cd.exitTime],"TCL":self.cd.TCL})
            return msg_obj
        elif actorX.state == "EXIT":
            msg_obj = msg(actorX.id,"CLEAR")
            return msg_obj

    def setup(self,egoX,worldX):
        # self.cd = cd.conflictDetection("timeSlot",[self.err]).obj
        self.cd = cd.conflictDetection("gridCell",[worldX.inter_location,4,16,self.err]).obj
        self.cd.setup(egoX,worldX)

class TEP_fix:
    #! Fix deadlocks by changing logic, vehicles are removed from waitlist they have lower Priority 
    def __init__(self,err,policy):
        self.err = err
        self.wait =[]
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
                            self.wait.remove(msg.idSend)
                    else:
                        if msg.idSend not in self.wait:
                            self.wait.append(msg.idSend)
                else: 
                    continue
            elif msg.mtype == "CLEAR":
                if msg.idSend in self.wait:
                    self.wait.remove(msg.idSend)

    def outbox(self,actorX):
        if actorX.state == "ENTER" or actorX.state == "CROSS":
            msg_obj = msg(actorX.id,"STOP",{"timeSlot":[self.cd.arrivalTime,self.cd.exitTime],"TCL":self.cd.TCL})
            return msg_obj
        elif actorX.state == "EXIT":
            msg_obj = msg(actorX.id,"CLEAR")
            return msg_obj

    def setup(self,egoX,worldX):
        # self.cd = cd.conflictDetection("timeSlot",[self.err]).obj
        # TODO Grid TCL is more like MP-IP fix later
        self.cd = cd.conflictDetection("gridCell",[worldX.inter_location,4,16,self.err]).obj
        self.cd.setup(egoX,worldX)

class MPIP:
    def __init__(self,err,policy):
        self.err = err
        self.wait ={}
        self.pp = priorityPolicy(policy)

    def resolve(self,egoX,worldX):
        # [self.ttArrival, self.ttExit] = self.cd.predictTimes(egoX,worldX)
        if len(worldX.msg.inbox) == 0:
            return 0
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.mtype == "ENTER" or msg.mtype == "CROSS":
                actorX = worldX.actorDict.dict.get(msg.idSend)
                TIC = self.cd.detect(egoX,worldX,msg)
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
            elif msg.mtype == "EXIT":
                if msg.idSend in self.wait:
                    del self.wait[msg.idSend]

    def outbox(self,actorX):
        if actorX.state == "ENTER":
            msg_obj = msg(actorX.id,"ENTER",{"timeSlot":[self.cd.arrivalTime,self.cd.exitTime],"TCL":self.cd.TCL})
            return msg_obj
        if actorX.state == "CROSS":
            msg_obj = msg(actorX.id,"CROSS",{"timeSlot":[self.cd.arrivalTime,self.cd.exitTime],"TCL":self.cd.TCL})
            return msg_obj
        elif actorX.state == "EXIT":
            msg_obj = msg(actorX.id,"EXIT")
            return msg_obj

    def setup(self,egoX,worldX):
        self.cd = cd.conflictDetection("gridCell",[worldX.inter_location,4,16,self.err]).obj
        self.cd.setup(egoX,worldX)

class AMPIP:
    def __init__(self,err,policy):
        self.err = err
        self.wait ={}
        self.pp = priorityPolicy(policy)

    def resolve(self,egoX,worldX):
        # [self.ttArrival, self.ttExit] = self.cd.predictTimes(egoX,worldX)
        if len(worldX.msg.inbox) == 0:
            return 0
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.mtype == "ENTER" or msg.mtype == "CROSS":
                actorX = worldX.actorDict.dict.get(msg.idSend)
                TIC = self.cd.detect(egoX,worldX,msg)
                [bool,TIC] = self.cd.detect(egoX,worldX,msg)
                if bool:
                    pOrder = self.pp.order([egoX,actorX])
                    if pOrder[0].id == egoX.id:
                        if msg.idSend in self.wait:
                            del self.wait[msg.idSend]
                    else:
                        [tAi,tEi] = self.cd.cellTimes(TIC,egoX)
                        [tAj,tEj] = actorX.cr.cd.cellTimes(TIC,actorX)
                        if msg.idSend not in self.wait:
                            if tEi > tAj:
                                self.wait[msg.idSend] = TIC
                            else:
                                # print(egoX.id," Going before ", msg.idSend)
                                continue
                else: 
                    continue
            elif msg.mtype == "EXIT":
                if msg.idSend in self.wait:
                    del self.wait[msg.idSend]

    def outbox(self,egoX):
        if egoX.state == "ENTER":
            msg_obj = msg(egoX.id,"ENTER",{"timeSlot":[self.cd.arrivalTime,self.cd.exitTime],"TCL":self.cd.TCL})
            return msg_obj
        if egoX.state == "CROSS":
            msg_obj = msg(egoX.id,"CROSS",{"timeSlot":[self.cd.arrivalTime,self.cd.exitTime],"TCL":self.cd.TCL})
            return msg_obj
        elif egoX.state == "EXIT":
            msg_obj = msg(egoX.id,"EXIT")
            return msg_obj

    def setup(self,egoX,worldX):
        self.cd = cd.conflictDetection("gridCell",[worldX.inter_location,4,16,self.err]).obj
        self.cd.setup(egoX,worldX)

class DCR:
    def __init__(self,err,policy):
        self.err = err
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
            # TODO Fix to be message propagated
            # Get TICL and traj info from front vehicle
            if egoX.info.get("idFront") != None:
                TICL = egoX.cr.cd.TICL(worldX.actorDict.dict.get(egoX.info.get("idFront")).cr.cd.TCL)
                traj = worldX.actorDict.dict.get(egoX.info.get("idFront")).cr.cd.traj
            else: # error when idFront is none therefore try except
                TICL = []
                traj = []
            self.wait[egoX.info.get("idFront")] = ["Queue",TICL,traj]
            return 0
        elif egoX.state == "OL":
            return 0 

        for msg in worldX.msg.inbox[egoX.id]:
            if msg.content.get("state") in ["NAN","IL"]:
                continue
            elif msg.content.get("state") == "OL":
                if msg.idSend in self.tmp:
                    del self.tmp[msg.idSend]
                if msg.idSend in self.wait:
                    del self.wait[msg.idSend]
                continue
            (sptBool,tmpBool,TICL) = self.cd.detect(egoX,worldX,msg)
            # If Ego has a spatial conflict with msg sender
            if sptBool:
                # If msg sender has temporal advantage over Ego
                if tmpBool == 0:
                    self.tmp[msg.idSend] = msg.content.get("state")
                    actorX = worldX.actorDict.dict.get(msg.idSend)
                    pOrder = self.pp.order([egoX,actorX])
                    if self.dd.yieldSearch(egoX,msg) == 0 or pOrder[1].id == egoX.id:
                        self.wait[actorX.id] = [msg.content.get("state"),TICL,msg.content.get("traj")]
                # If Ego has temporal advantage over msg sender
                if tmpBool == 1:  
                    actorX = worldX.actorDict.dict.get(msg.idSend)
                    pOrder = self.pp.order([egoX,actorX])
                    if self.dd.yieldSearch(egoX,msg) == 1 and pOrder[0].id == egoX.id:
                        self.wait[msg.idSend] = [msg.content.get("state"),TICL,msg.content.get("traj")]

        for idYield in self.wait:
            for cell in self.wait.get(idYield)[1]:
                # Prediction of time actor leaves cell + err for robustness
                TOutAct = self.wait.get(idYield)[2].get(cell)[1] + self.err
                # Current time for ego to enter cell
                TinEgo = self.cd.traj[cell][0]
                # if the yielding changes the Ego vehicles times
                if TOutAct > TinEgo:
                    delay = TOutAct-TinEgo
                    # if self.cd.TCL.index(cell) < len(self.cd.TCL) - 1:
                    # Amount of cells that need to be updated with the new delay
                    cellIndex = self.cd.TCL.index(cell)
                    delayedCells = len(self.cd.TCL) - cellIndex
                    for i in range(delayedCells):
                        cell = self.cd.TCL[i+cellIndex]
                        self.cd.traj[cell] = (self.cd.traj[cell][0] + delay, self.cd.traj[cell][1] + delay)
                        
                # update all other sequential cells as well
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
        #content[wait] = self.wait
        msg_obj = msg(egoX.id,"COM",content)
        return msg_obj

    def setPriority(self,score):
        self.priorityScore = score


    def setup(self,egoX,worldX):
        self.cd = cd.conflictDetection("conflictZones",[worldX.inter_location,2,8,self.err]).obj
        self.cd.setup(egoX,worldX)
        self.setPriority(0)
        
        

