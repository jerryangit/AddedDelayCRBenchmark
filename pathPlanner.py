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
    def __init__(self,worldX):
        self.worldX = worldX
        self.waypoints = worldX.map.get_waypoint(ego.get_location())
 
    def plan(self,arg,actorX):
        cases = {
            "discretePaths": self.discretePaths,
        }
        fnc = cases.get(arg)
        return fnc(actorX)

    def discretePaths(self,actorX):
        # Hardcoded
        
        path = []
        return path
