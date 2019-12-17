#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Contains helper functions for the actor, e.g. messaging, lowlevel detection etc.

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

    def send(self, msg,recipient):
        self.inbox[recipient].append(msg)

    def broadcast(self,ego,msg,range=100):
        self.cloud.append([ego,range,msg])

    def receive(self, ego):
        for msg in self.cloud:
            if ego.get_location().distance(carla.Location(msg[0].get_location())) < msg[1]:
                self.inbox[ego.id].append(msg)

class worldX:
    # Class containing world info needed for the vehicles, is available for every vehicle
    def __init__(self,world):
        self.world = world
        self.map = world.get_map()
    def update(self,actorDict):
        self.actorDict = actorDict

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
    def __init__(self,ego,model,dt):
        self.ego = ego
        self.id = ego.id
        self.arrivalTime = 0
        self.departureTime = 0
        self.updateParameters(ego)
        self.dt = dt
        if model == 0: 
            self.doubleS()
        if model == 1: 
            self.unicycle()
        if model == 2: 
            self.bicycle()

    def updateParameters(self,ego):
        self.velocity = ego.get_velocity()
        self.vel_norm = np.linalg.norm([self.velocity.x,self.velocity.y])
        self.location = ego.get_transform().location
        self.orientation = ego.get_transform().rotation
        self.predictPath()
        self.predictTimes()

    def predictPath(self): 
        # Maybe separate class for path planning?
        self.sArrival = 0
        self.sExit = 0
        return [self.sArrival,self.sExit]

    def predictTimes(self):
        self.arrivalTime = self.sArrival / self.vel_norm
        self.exitTime = self.sExit / self.vel_norm
        return [self.arrivalTime,self.exitTime]

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
        

