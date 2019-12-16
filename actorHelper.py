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

class worldInfo:
    def __init__(self,world,actor_list):
        self.world = world
        self.actor_list = actor_list

class info:
     def __init__(self, index, waypoints,actor_list):
        self.index = index
        self.waypoints = waypoints
        self.actor_list = actor_list

class msgInterface:
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

