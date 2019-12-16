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
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np

class conflictResolution:
    def __init__(self,protocol):
        self.protocol = protocol

    def resolve(self,actor,inbox,info):
        self.switch(self.protocol)
  
    def switch(self,arg):
        cases = {
            "TEP": self.TEP,
            "FCFS": self.FCFS,
        }
        fnc = cases.get(arg,"No valid method found")
        fnc()

    def TEP(self):
        pass
    def FCFS(self):
        pass



