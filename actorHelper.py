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
            if msg[0].id == id:
                self.cloud.remove(msg)
        del self.inbox[id]

    def clearCloud(self):
        self.cloud = []

    def send(self, msg,recipient):
        self.inbox[recipient].append(msg)

    def broadcast(self,ego,msg,range=100):
        if msg != None:
            self.cloud.append([ego,range,msg])

    def receive(self, ego):
        # ! Hard reset inbox each tick
        self.inbox[ego.id] = []
        for msg in self.cloud:
            if ego.id != msg[0].id and ego.get_location().distance(carla.Location(msg[0].get_location())) < msg[1]:
                self.inbox[ego.id].append(msg[2])

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
    def __init__(self,ego,model,dt,dest,velRef,VIN):
        self.route = []
        self.routeIndex = 0
        self.dest = dest
        self.sTraversed = 0
        self.state = "NAN"
        self.velRef = velRef
        self.cr = []
        self.updateParameters(ego,dt)
        self.id = ego.id
        self.VIN = VIN
        if model == 0: 
            self.doubleS()
        if model == 1: 
            self.unicycle()
        if model == 2: 
            self.bicycle()

    def updateParameters(self,ego,dt):
        self.ego = ego
        self.dt = dt
        self.velocity = ego.get_velocity()
        self.vel_norm = np.linalg.norm([self.velocity.x,self.velocity.y])
        self.location = ego.get_transform().location
        self.rotation = ego.get_transform().rotation
        self.sTraversed += self.dt*self.vel_norm

    def discreteState(self,state):
        self.state = state

    def doubleS(self):
        # Double integrator for x,y
        # States:   [x,xdot,y,ydot]'  
        # Input:    [xddot,yddot]'
        A = np.array([[0,1,0,0],[0,0,0,0],[0,0,0,1],[0,0,0,0]])
        B = np.array([[0,0],[1,0],[0,0],[0,1]])
        C = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        D = np.array([[0,0],[0,0],[0,0],[0,0]])
        self.ss = scipy.signal.StateSpace(A,B,C,D)

    def unicycle(self):
        # Unicycle Model
        # States:   [x,y,theta]'  
        # Input:    [v,thetadot]'
        theta = 0   # !Linearized! around theta = 0, fix later
        A = np.array([[0,0,0],[0,0,0],[0,0,0]])
        B = np.array([[np.cos(theta),0],[np.sin(theta),0],[0,1]])
        C = np.array([[1,0,0],[0,1,0],[0,0,1]])
        D = np.array([[0,0],[0,0],[0,0]])
        self.ss = scipy.signal.StateSpace(A,B,C,D)

    def bicycle(self):
        #>>>>>>>>TODO<<<<<<<<<<
        # Double integrator for x,y
        # States:   [x,xdot,y,ydot]'  
        # Input:    [xddot,yddot]'
        A = np.array([[0,1,0,0],[0,0,0,0],[0,0,0,1],[0,0,0,0]])
        B = np.array([[0,0],[1,0],[0,0],[0,1]])
        C = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        D = np.array([[0,0],[0,0],[0,0],[0,0]])
        self.ss = scipy.signal.StateSpace(A,B,C,D)
        

