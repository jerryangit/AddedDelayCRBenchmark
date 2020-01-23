#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
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

class deadlockDetection:
    def __init__(self,method,para=[]):
        self.method = method
        self.obj = self.switchCreate(method,para)

    def switchCreate(self,arg,para):
        cases = {
            "DCRgraph": DCRgraph,
        }
        fnc = cases.get(arg)
        if isinstance(para, list):
            return fnc(*para)
        else:
            return fnc(para)

class DCRgraph:
    def __init__(self):
        self.edgeDict = {}
        self.stateDict = {}
        self.queue = []
        self.history = []
    def addEdgeList(self,id,edgeList):
        self.edgeDict[id] = edgeList
    
    def setState(self,id,state):
        self.stateDict[id] = state

    def yieldSearch(self,egoTmp,egoId,actId,egoState,actState):
        # clear history and initialize queue with own wait list
        self.history = [egoId]
        self.queue = list(egoTmp.keys())
        if egoState == actState:
            # Go through the queue
            while len(self.queue) > 0:
                id = self.queue[0]
                # If the id in the queue is the actorX.id 
                if id == actId:
                    return 1 # egoX yield for actorX (in)directly

                # If the id in the queue has the same state as ego => explore graph
                try:
                    if id != egoId and self.stateDict[id] == egoState:
                        # Add wait list of id to queue
                        for tmp_id in self.edgeDict[id]:
                            if tmp_id not in self.history and self.edgeDict[id].get(tmp_id) != "OL":
                                self.queue.append(tmp_id) 
                except:
                    print("error")
                self.history.append(id)
                self.queue.remove(id)
        elif egoState == "I" and actState == "FIL":
            # Go through the queue
            while len(self.queue) > 0:
                id = self.queue[0]
                # If the id in the queue is the actorX.id 
                if id == actId:
                    return 1 # egoX yield for actorX (in)directly
                # Add wait list of id to queue
                for tmp_id in self.edgeDict[id]:
                    if tmp_id not in self.history and self.edgeDict[id].get(tmp_id) != "OL":
                        self.queue.append(tmp_id) 
                self.history.append(id)
                self.queue.remove(id)
        return 0
