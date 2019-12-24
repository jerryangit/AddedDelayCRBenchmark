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
            "TimeSpent": self.TimeSpent
        }
        fnc = cases.get(self.policy)
        #* 1 = ego holds priorty, 0 = ego has to yield
        return fnc(para)

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
        #TODO Grid TCL is more like MP-IP fix later
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

class DCR:
    def __init__(self,err,policy):
        self.err = err
        self.wait ={}
        self.pp = priorityPolicy(policy)
        self.dd = dd.deadlockDetection("Tie")

    def resolve(self,egoX,worldX):
        # [self.ttArrival, self.ttExit] = self.cd.predictTimes(egoX,worldX)
        if len(worldX.msg.inbox) == 0:
            return 0
        if egoX.state == "IL":
            self.wait[idFront] = "Queue"
        for msg in worldX.msg.inbox[egoX.id]:
            (sptBool,tmpBool) = self.cd.detect(egoX,worldX,msg)
            # If Ego has a spatial conflict with msg sender
            if sptBool:
                # If msg sender has temporal advantage over Ego
                if tmpBool == 0:
                    actorX = worldX.actorDict.dict.get(msg.idSend)
                    pOrder = self.pp.order([egoX,actorX])
                    if No Tie(ego,act) or pOrder[1].id == egoX.id:
                        self.wait[egoX.id] = 1
                # If Ego has temporal advantage over msg sender
                if tmpBool == 1:  
                    actorX = worldX.actorDict.dict.get(msg.idSend)
                    pOrder = self.pp.order([egoX,actorX])
                    if Tie(act,ego) and pOrder[0].id == egoX.id:
                        self.wait[egoX.id] = 1
    def outbox(self,actorX):
        pass

    def setup(self,egoX,worldX):
        self.cd = cd.conflictDetection("conflictZones",[worldX.inter_location,2,8,self.err]).obj
        self.cd.setup(egoX,worldX)

