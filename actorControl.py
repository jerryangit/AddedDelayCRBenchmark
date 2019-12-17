#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import actorHelper as ah
import conflictDetection as cd
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

def simpleControl(ego,worldX): 
    velocityPID(ego,8.33)
    cd_obj = cd.conflictDetection("coneDetect",ego,[6,0.185*np.pi,5]).obj
    for actor in worldX.actorDict.actor_list:
        if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 12 : 
            if cd_obj.detect(actor):
                ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0))
                # print(ego.id,' has to Emergency Break because of  ', actor.id)

def velocityPID(ego,vref):
    v = np.linalg.norm([ego.get_velocity().x,ego.get_velocity().y])
    kp = 0.1
    u = kp*(vref-v)
    ego.apply_control(carla.VehicleControl(throttle=u, steer=0.0,brake = 0.0))
