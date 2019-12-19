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
def TEPControl(egoX,worldX):
    # TODO Fix arrival time when near intersection, shouldn't increase as it is FCFS
    ego = egoX.ego
    if egoX.location == carla.Location(0,0,0):
        return "NAN"
    u_v = velocityPID(ego,egoX.velRef)
    u_theta = pathFollow(egoX,worldX)
    ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0.0))
    cd_obj = cd.conflictDetection("coneDetect",[4.5,0.185*np.pi,5]).obj
    for actor in worldX.actorDict.actor_list:
        if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 12 : 
            if cd_obj.detect(ego,actor):
                ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0))
    if len(egoX.cr.wait) > 0 and ego.get_location().distance(worldX.inter_location) <= worldX.inter_bounds + 3 :
        if egoX.cr.cd.arr[0] != 1:
            egoX.cr.cd.arr = [1,egoX.cr.cd.arrivalTime]
        ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0))        
    state = egoX.state
    if egoX.state == "NAN":
        state = "ENTER"
    elif egoX.state == "ENTER" and ego.get_location().distance(worldX.inter_location) <= worldX.inter_bounds:
        if egoX.cr.cd.arr[0] != 1:
            egoX.cr.cd.arr = [1,egoX.cr.cd.arrivalTime]
        state = "CROSS"
    elif egoX.state == "CROSS" and ego.get_location().distance(worldX.inter_location) >= worldX.inter_bounds+3:
        if egoX.cr.cd.ext[0] != 1:
            egoX.cr.cd.ext = [1,egoX.cr.cd.exitTime]
        state = "EXIT"
    return state


def simpleControl(ego,worldX): 
    velocityPID(ego,8.33)
    cd_obj = cd.conflictDetection("coneDetect",[5,0.185*np.pi,5]).obj
    for actor in worldX.actorDict.actor_list:
        if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 12 : 
            if cd_obj.detect(ego,actor):
                ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0))
                # print(ego.id,' has to Emergency Break because of  ', actor.id)

def velocityPID(ego,vref):
    v = np.linalg.norm([ego.get_velocity().x,ego.get_velocity().y])
    kp_v = 0.1
    u_v = kp_v*(vref-v)
    return u_v

def pathFollow(egoX,worldX):
    kp_theta = 2.65
    dist = []
    for i in range(4):
        try:
            dist.append([egoX.location.distance(egoX.route[egoX.routeIndex+i][0].transform.location),i])
        except:
            pass
    egoX.routeIndex = egoX.routeIndex+sorted(dist)[0][1]
    theta_way = wrap((np.pi*egoX.route[egoX.routeIndex][0].transform.rotation.yaw)/180)
    theta_ego = wrap((np.pi*egoX.rotation.yaw)/180)
    u_theta = kp_theta*wrap(theta_way-theta_ego)
    # print("ID: ",egoX.id,"|U_Theta: ", u_theta)
    return u_theta


def wrap(angle):
    while np.abs(angle) > np.pi:
        if angle > np.pi:
            angle = angle - 2 * np.pi
        elif angle < -np.pi:
            angle = angle + 2 * np.pi
    return angle