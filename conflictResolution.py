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
    
class conflictResolution:
    def __init__(self,method,ego,para=[]):
        self.method = method
        self.para = para
        self.obj = self.switchCreate(method,ego,para)
  
    def switchCreate(self,arg,ego,para):
        cases = {
            "TEP": TEP,
            "FCFS": FCFS,
        }
        fnc = cases.get(arg,"No valid method found")
        return fnc(ego,*para)

class TEP:
    def __init__(self):
        pass
    def resolve(self,ego,msg_obj,info):
        cd_obj = cd.conflictDetection("timeSlot",0).obj
        for msg in msg_obj.inbox:
            if msg.mtype == "STOP":
                # cd_obj.detect(A0,A1,B0,B1)
                pass

class FCFS:
    def __init__(self): 
        pass
