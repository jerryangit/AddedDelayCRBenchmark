import actorHelper as ah
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

class info:
     def __init__(self, index, waypoints,actor_list):
        self.index = index
        self.waypoints = waypoints
        self.actor_list = actor_list


def simpleControl(ego,info): 

    velocityPID(ego,8.33)

    cd = ah.coneDetect(ego,radius=6,angle=0.175*np.pi,actorSamples=5)
    j = 0
    for actor in info.actor_list:
        if j != info.index and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 10 : 
            if cd.coneDetect(actor):
                ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0))
                print(ego.id,' has to Emergency Break because of  ', actor.id)
        j += 1        



def velocityPID(ego,vref):
    v = np.linalg.norm([ego.get_velocity().x,ego.get_velocity().y])
    kp = 0.2
    u = kp*(vref-v)
    ego.apply_control(carla.VehicleControl(throttle=u, steer=0.0,brake = 0.0))