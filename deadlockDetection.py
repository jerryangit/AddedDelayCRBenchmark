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
    def addEdgeList(self,ID,edgeList):
        # edgeList is the tmp of the ID, where tmp includes vehicles which have a temporal advantage over it
        self.edgeDict[ID] = edgeList
    
    def setState(self,ID,state):
        self.stateDict[ID] = state

    def yieldSearch(self,egoTmp,egoID,actID,egoState,actState):
        # clear history and initialize queue with own wait list
        self.history = [actID]
        actTmp = self.edgeDict.get(actID)
        if actTmp != None:
            self.queue = list(actTmp.keys())
        else:
            # if actTmp = none, means that no vehicle holds temporal advantage over actor
            return 0
        # For a tie to exist actID has to have temporal advantage over egoID
        if actID in egoTmp:
            if egoState == actState:
                # Go through the queue
                while len(self.queue) > 0:
                    ID = self.queue[0]
                    # If the ID in the queue is the ego.ID 
                    if ID == egoID:
                        return 1 # egoID holds temporal advantage over actorID (in)directly
                    # If the ID in the queue has the same state as ego => explore graph
                    try:
                        if ID != actID and self.stateDict[ID] == egoState:
                            # Add tmp list of ID to queue
                            for tmp_ID in self.edgeDict[ID]:
                                if tmp_ID not in self.history and self.edgeDict[ID].get(tmp_ID) != "OL":
                                    self.queue.append(tmp_ID) 
                    except:
                        print("error")
                    self.history.append(ID)
                    self.queue.remove(ID)
            elif egoState == "I" and actState == "FIL":
                # Go through the queue
                while len(self.queue) > 0:
                    ID = self.queue[0]
                    # If the ID in the queue is the actorX.ID 
                    if ID == egoID:
                        return 1 # egoID holds temporal advantage over actorID (in)directly
                    # Add tmp list of ID to queue
                    for tmp_ID in self.edgeDict[ID]:
                        if tmp_ID not in self.history and self.edgeDict[ID].get(tmp_ID) != "OL":
                            self.queue.append(tmp_ID) 
                    self.history.append(ID)
                    self.queue.remove(ID)
        return 0
