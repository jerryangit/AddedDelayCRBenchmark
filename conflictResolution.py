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
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np

class msg:
    def __init__(self,mtype="N/A",content={}):
        self.mtype = mtype
        self.content = content
    

class priorityPolicy:
    def __init__(self,policy,actorInfo):
        self.policy = policy
        self.actorInfo = actorInfo

    def order(self,arg,para):
        cases = {
            "FCFS": self.FCFS,
        }
        fnc = cases.get(arg)
        return fnc(*para)

    def FCFS(self,actorList):
        return actorList.sort(key = (self.arrivalTime,self.actorID))

    def tieBreaker(self,actorList):
        return actorList.sort(key = self.actorID)

    def actorID(self,actor):
        return actor.id

    def arrivalTime(self,actor):
        obj = self.actorInfo.dict.get(actor.id)
        return obj.arrivalTime
    

class conflictResolution:
    def __init__(self,method,ego,worldX,para=[]):
        self.method = method
        self.obj = self.switchCreate(method,ego,worldX,para)
  
    def switchCreate(self,arg,ego,worldX,para):
        cases = {
            "TEP": TEP,
            "MPIP": MPIP,
        }
        fnc = cases.get(arg)
        return fnc(ego,worldX,*para)

class TEP:
    def __init__(self,ego,worldX,para=[]):
        pass
    def resolve(self,ego,msg_obj,info):
        cd_obj = cd.conflictDetection("timeSlot",0).obj
        for msg in msg_obj.inbox:
            if msg.mtype == "STOP":
                # if cd_obj.detect(A0,A1,B0,B1):
                    
                pass

class RR:
    def __init__(self): 
        pass


class MPIP:
    def __init__(self): 
        pass

