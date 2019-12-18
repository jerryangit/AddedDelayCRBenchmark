#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import conflictDetection as cd
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
        }
        fnc = cases.get(self.policy)
        #* 1 = ego holds priorty, 0 = ego has to yield
        return fnc(para)

    def FCFS(self,actorList):
        actorList.sort(key = lambda x: (x.cr.arrivalTime,x.id))
        return actorList

    def actorID(self,actorX):
        return actorX.id

    def arrivalTime(self,actorX):
        return actorX.cr.arrivalTime
    

class conflictResolution:
    def __init__(self,method,para=[]):
        self.method = method
        self.obj = self.switchCreate(method,para)

    def switchCreate(self,arg,para):
        cases = {
            "TEP": TEP,
            "MPIP": MPIP,
        }
        fnc = cases.get(arg)
        return fnc(*para)

class TEP:
    def __init__(self,err,policy,para=[]):
        self.id = -1
        self.para = para
        self.err = err
        self.wait =[]
        self.arrivalTime = np.inf
        self.exitTime = np.inf
        self.pp = priorityPolicy(policy)
    def resolve(self,egoX,worldX):
        cd_obj = cd.conflictDetection("timeSlot",self.err).obj
        aeTimes = cd_obj.predictTimes(egoX,worldX)
        self.arrivalTime = aeTimes[0]
        self.exitTime = aeTimes[1]

        if len(worldX.msg.inbox) == 0:
            return 0
        for msg in worldX.msg.inbox[egoX.id]:
            if msg.mtype == "STOP" and msg.idSend not in self.wait:
                actorX = worldX.actorDict.dict.get(msg.idSend)
                if cd_obj.detect(egoX,actorX,worldX):
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
            msg_obj = msg(actorX.id,"STOP",{"arrivalTime":self.arrivalTime})
            return msg_obj
        elif actorX.state == "EXIT":
            msg_obj = msg(actorX.id,"CLEAR")
            return msg_obj

    def tick(self):
        pass


class RR:
    def __init__(self): 
        pass


class MPIP:
    def __init__(self): 
        pass

