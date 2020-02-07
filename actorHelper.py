#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Contains helper functions for the actor, e.g. messaging, lowlevel detection etc.
import pathPlanner as pp
import glob
import os
import sys
import csv
import datetime
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import numpy as np
import scipy.signal


class msgInterface:
    # Class which facilitates communication between vehicles, vehicles shouldn't be able to access eachothers thoughts directly
    def __init__(self,inbox={},cloud=[]):
        self.inbox = inbox
        self.cloud = cloud 
    
    def clear(self,id):
        for msg in self.cloud:
            if msg[0] == id:
                self.cloud.remove(msg)
        del self.inbox[id]

    def clearCloud(self):
        self.cloud = []

    def send(self, msg,recipient):
        self.inbox[recipient].append(msg)

    def broadcast(self,id,location,msg,range=250):
        if msg != None:
            self.cloud.append([id,location,range,msg])

    def receive(self, egoX):
        # ! Hard reset inbox each tick
        self.inbox[egoX.id] = []
        for msg in self.cloud:
            if egoX.id != msg[0] and egoX.location.distance(msg[1]) < msg[2]:
                self.inbox[egoX.id].append(msg[3])

class worldX:
    # Class containing world info needed for the vehicles, is available for every vehicle
    def __init__(self,world,inter_location,inter_bounds,tick,hopres):
        self.world = world
        self.map = world.get_map()
        self.msg = msgInterface()
        self.inter_location = inter_location
        self.inter_bounds = inter_bounds
        self.initialTS = tick.timestamp
        self.tock(tick)
        self.hopres = hopres
        # self.dict = {}
    def update(self,actorDict):
        self.actorDict = actorDict
    def tock(self,tick):
        self.tick = tick

class actorDict:
    # Class dictionary made up of vehicle objects attributed to their id. 
    def __init__(self,actor_list=[],info={}):
        self.dict = info
        self.actor_list = actor_list
    def addKey(self,key,value):
        self.dict[key] = value
    def addApp(self,key,value):
        self.dict[key].append(value)

class actorX:
    # TODO optimize calculations, e.g. calc location once per loop save in this class object
    # Class containing information for each vehicle, used to unify required data, can't modify cpp actor class
    def __init__(self,ego,model,dt,spwnTime,spwn,spwnid,dest,destid,velRef,VIN,spwnNr):
        # Conflict Resolution Object
        self.cr = []
        # Control Resolution Object
        self.co = []
        # Actor object
        self.ego = ego
        # Info dictionary for various purposes
        self.info = {}
        # Route list of waypoints
        self.route = []
        # Current index of the route
        self.routeIndex = 0
        # Current nearest waypoint
        self.waypoint = []
        # Spawn location
        self.spwn = spwn
        # Spawn ID
        self.spwnid = spwnid
        # Spawn Time
        self.spwnTime = spwnTime
        # Destination Location
        self.dest = dest
        # Destination ID
        self.destid = destid
        # Traversed distance
        self.sTraversed = 0
        # Current discrete State
        self.state = "NAN"
        # Reference velocity
        self.velRef = velRef
        # ID
        self.id = ego.id
        # VIN (Used for policy only)
        self.VIN = VIN
        # onTickList:
        self.onTickList = []
        # amin
        self.aMin = 9
        # amax 
        self.aMax = 2.5
        # spwnNr
        self._spwnNr = spwnNr
        self.updateParameters(dt)

    def infoSet(self,key,value):
        self.info[key] = value

    def updateParameters(self,dt):
        self.dt = dt
        self.velocity = self.ego.get_velocity()
        # TODO 3D norm?
        self.velNorm = np.linalg.norm([self.velocity.x,self.velocity.y])
        self.location = self.ego.get_transform().location
        self.rotation = self.ego.get_transform().rotation
        # Rough Integration of distance
        self.sTraversed += self.dt*self.velNorm
        for fnc in self.onTickList:
            fnc()
        self.aMax = 2 + (0.25 * self.velNorm -1)**2+1

    def onTick(self,fnc):
        self.onTickList.append(fnc)        

    def discreteState(self,state):
        self.state = state

