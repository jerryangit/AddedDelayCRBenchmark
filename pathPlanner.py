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
import scipy as sp

class pathPlanning:
    def __init__(self,method,para=[]):
        self.method = method
        self.para = para
        self.obj = self.switchCreate(method,para)
 
    def switchCreate(self,arg,para):
        cases = {
            "discretePaths": discretePaths,
        }
        fnc = cases.get(arg,"No valid method found")
        return fnc(para)

class discretePaths:
    def __init__(self,map):
        self.map = map
        self.waypoints = map.waypoints
    def path():
        pass

    #                w = map.get_waypoint(actor.get_location())



