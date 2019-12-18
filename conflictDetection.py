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
    sys.path.append(glob.glob('../carla/dist/carla-*%d.5-%s.egg' % (
        sys.version_info.major,
        # sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np

class conflictDetection:
    def __init__(self,method,ego,para=[]):
        self.method = method
        self.para = para
        self.obj = self.switchCreate(method,ego,para)

    def switchCreate(self,arg,ego,para):
        cases = {
            "coneDetect": coneDetect,
            "timeSlot": timeSlot,
        }
        fnc = cases.get(arg,"No valid method found")
        return fnc(ego,*para)

class timeSlot:
    def __init__(self,error=0):
        self.error = error
        self.arrivalTime = -np.inf
        self.exitTime = np.inf
    def detect(self,egoX,actorX,worldX): 
        # slot is a list with [0], being the start of the slot and [1] being the end of the slot
        #TODO maybe move displacement elsewhere might be intensive
        ego_t = self.predictTimes(egoX,worldX)
        actor_t = self.predictTimes(actorX,worldX)
        if ego_t[1] > actor_t[0] + self.error and ego_t[0] < actor_t[1] - self.error:
            return 1
        if actor_t[1] > ego_t[0] + self.error and actor_t[0] < ego_t[1] - self.error:
            return 1
        return 0 

    def displacement(self,egoX,worldX):
    #TODO detect which waypoint and which displacement enters and exits bounds of intersection as defined somewhere else?
        sArrival = np.inf
        sExit = np.inf
        for wr in egoX.path:
            if wr[0].transform.location.distance(worldX.inter_location)<=worldX.inter_bounds and sArrival == np.inf:
                sArrival = wr[0].s
            if wr[0].transform.location.distance(worldX.inter_location)>=worldX.inter_bounds and sArrival != np.inf:
                sExit = wr[0].s
                return [sArrival,sExit]
        return [sArrival,sExit]

    def predictTimes(self,egoX,worldX):
        sList = self.displacement(egoX,worldX)
        self.arrivalTime = (sList[0]-egoX.disp) / egoX.velRef
        self.exitTime = (sList[1]-egoX.disp) / egoX.velRef
        return [self.arrivalTime,self.exitTime]


class coneDetect:
    def __init__(self,ego,radius=2.5,angle=0.33*np.pi,actorSamples=5):
        # Sets ego vehicle
        self.ego = ego
        # Sets Radius for the cone from center of own vehicle
        self.radius = radius
        # Sets cone angle width 
        self.angle = angle
        # Sets amount of interpolating samples per 2 sides, actual amount is 2x to enforce symmetry 
        self.actorSamples = actorSamples
    def detect(self,actor):
        smpArr = self.genSamples(actor)
        for i in range(smpArr.shape[0]):
            # relative vector from ego vehicle to sample
            vec = smpArr[i]- [ self.ego.get_location().x , self.ego.get_location().y ]
            r = np.linalg.norm( vec )
            # Note: Coordinate frame carla has left handed coordinate frame!!
            phi0 = (self.ego.get_transform().rotation.yaw * np.pi)/ 180 
            phi = np.arctan2( vec[1] , vec[0] ) - phi0
            if r < self.radius and np.absolute( phi ) < 0.5 * self.angle:
                return 1           
        return 0
    def genSamples(self,actor):
        # TODO integrate some of these into helper functions
        # Determine lenght of bbox in x and y directions
        xLength = actor.bounding_box.extent.x * 2
        yLength = actor.bounding_box.extent.y * 2
        # Def amount of sample along x and y edges, rounded to int
        xSamples = int( round( self.actorSamples * ( xLength / ( xLength + yLength ) ) ) )
        ySamples = self.actorSamples - xSamples
        # Find actor orientation
        actorF = actor.get_transform()
        # Initialize array with corners, rows = samples, columns = x,y
        smpArr = np.array([ [actor.bounding_box.extent.x , actor.bounding_box.extent.y] ,\
            [actor.bounding_box.extent.x,-actor.bounding_box.extent.y],\
            [-actor.bounding_box.extent.x,actor.bounding_box.extent.y],\
            [-actor.bounding_box.extent.x,-actor.bounding_box.extent.y] ])
        # Interpolate sample between corners
        for i in range(xSamples):
            # Compute relative x location for sample i from - to + 
            xi = -actor.bounding_box.extent.x + ( i + 1 ) * ( ( 2 * actor.bounding_box.extent.x ) / ( xSamples + 1) )
            # Append interpolated sample i on both sides 
            smpArr = np.append(smpArr,[ [ xi, actor.bounding_box.extent.y ] , [ xi, -actor.bounding_box.extent.y ] ] , 0 )
        for i in range(ySamples):
            # Compute relative y location for sample i from - to + 
            yi = -actor.bounding_box.extent.y + ( i + 1 ) * ( ( 2 * actor.bounding_box.extent.y ) / ( ySamples + 1) )
            # Append interpolated sample i on both sides 
            smpArr = np.append(smpArr,[ [ actor.bounding_box.extent.x , yi ] , [ -actor.bounding_box.extent.x , yi ] ] , 0 )
        # Rotate array with vehicle orientation
        c = np.cos(np.pi*actorF.rotation.yaw/180)
        s = np.sin(np.pi*actorF.rotation.yaw/180)
        # Transposed rotation matrix due to rightside matrix multiplication 
        RT = np.array( [ [ c , s ] , [ -s, c ] ] ) 
        smpArr = np.matmul(smpArr,RT)
        # Move sample array to global coordinate frame
        smpArr = smpArr + [ actor.get_location().x , actor.get_location().y ]
        return smpArr

