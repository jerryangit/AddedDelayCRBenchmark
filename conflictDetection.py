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
    def __init__(self,method,para=[]):
        self.method = method
        self.para = para
        self.obj = self.switchCreate(method,para)

    def switchCreate(self,arg,para):
        cases = {
            "coneDetect": coneDetect,
            "timeSlot": timeSlot,
            "gridCell": gridCell,
        }
        fnc = cases.get(arg,"No valid method found")
        if isinstance(para, list):
            return fnc(*para)
        else:
            return fnc(para)

class timeSlot:
    def __init__(self,error=0):
        self.error = error
        self.arr = [0,0]
        self.ext = [0,0]
    def detect(self,egoX,worldX,actor_t): 
        # slot is a list with [0], being the start of the slot and [1] being the end of the slot
        ego_t = self.predictTimes(egoX,worldX)
        if ego_t[1] > actor_t[0] + self.error and ego_t[0] < actor_t[1] - self.error:
            return 1
        if actor_t[1] > ego_t[0] + self.error and actor_t[0] < ego_t[1] - self.error:
            return 1
        return 0 

    def predictTimes(self,egoX,worldX):
        if egoX.cr.cd.arr[0] != 1:
            self.arrivalTime = ( (self.sArrival-egoX.sTraversed) / egoX.velRef ) + worldX.tick.timestamp.elapsed_seconds
        else: 
            self.arrivalTime = egoX.cr.cd.arr[1]
        if egoX.cr.cd.ext[0] != 1:
            self.exitTime = ( (self.sExit-egoX.sTraversed) / egoX.velRef ) + worldX.tick.timestamp.elapsed_seconds
        else: 
            self.exitTime = egoX.cr.cd.ext[1]
        return [self.arrivalTime,self.exitTime]

    def sPathCalc(self,egoX,worldX):
        self.sArrival = np.inf
        self.sExit = np.inf
        self.sPath = []
        s = 0
        s0 = egoX.route[0][0].transform.location
        for wr in egoX.route:
            s = wr[0].transform.location.distance(s0) + s
            s0 = wr[0].transform.location
            if self.sArrival == np.inf:
                if wr[0].transform.location.distance(worldX.inter_location)<=worldX.inter_bounds:
                    self.sArrival = s
                    self.sPath.append([s,1,"enter"])
                else:
                    self.sPath.append([s,0,"pre"])
            elif self.sExit == np.inf:
                if s > self.sArrival + 2 and wr[0].transform.location.distance(worldX.inter_location)>=worldX.inter_bounds:
                    self.sExit = s
                    self.sPath.append([s,3,"exit"])
                else:
                    self.sPath.append([s,2,"cross"])
            else:
                self.sPath.append([s,4,"post"])
        return self.sPath

class gridCell:
    def __init__(self,center,resolution=4,size=16):
        self.center = center            # Carla.Location of intersection center
        self.resolution = resolution    # Ammount of cells per row/column
        self.size = size                # Distance in m between edges of grid        
        self.calcGrid()
    def detect(self,egoX,actorX,worldX): 
        pass
    def calcGrid(self):
        
        # for i_x in range(self.resolution+1):
        #         dx = self.size/self.resolution
        #         # top = [tl[0]+i_x*dx,tl[1]] 
        #         bot = [bl[0]+i_x*dx,bl[1]]
        #         x_list.append()
        # for i_y in range(self.resolution+1):
        #         dy = self.size/self.resolution
        #         lef = [tl[0],tl[1]+i_y*dy] 
        #         rig = [tr[0],tr[1]+i_y*dy]
        #         y_list.append()
        self.resolution

    def sPathCalc(self,egoX,worldX):
        self.sArrival = np.inf
        self.sExit = np.inf
        self.sPath = []
        s = 0
        s0 = egoX.route[0][0].transform.location
        for wr in egoX.route:
            s = wr[0].transform.location.distance(s0) + s
            s0 = wr[0].transform.location
            if self.sArrival == np.inf:
                if wr[0].transform.location.distance(worldX.inter_location)<=worldX.inter_bounds:
                    self.sArrival = s
                    self.sPath.append([s,1,"enter"])
                else:
                    self.sPath.append([s,0,"pre"])
            elif self.sExit == np.inf:
                if s > self.sArrival + 2 and wr[0].transform.location.distance(worldX.inter_location)>=worldX.inter_bounds:
                    self.sExit = s
                    self.sPath.append([s,3,"exit"])
                else:
                    self.sPath.append([s,2,"cross"])
            else:
                self.sPath.append([s,4,"post"])
        return self.sPath

class coneDetect:
    def __init__(self,radius=2.5,angle=0.33*np.pi,actorSamples=5):
        # Sets Radius for the cone from center of own vehicle
        self.radius = radius
        # Sets cone angle width 
        self.angle = angle
        # Sets amount of interpolating samples per 2 sides, actual amount is 2x to enforce symmetry 
        self.actorSamples = actorSamples
    def detect(self,ego,actor):
        smpArr = self.genSamples(actor)
        for i in range(smpArr.shape[0]):
            # relative vector from ego vehicle to sample
            vec = smpArr[i]- [ ego.get_location().x , ego.get_location().y ]
            r = np.linalg.norm( vec )
            # Note: Coordinate frame carla has left handed coordinate frame!!
            phi0 = (ego.get_transform().rotation.yaw * np.pi)/ 180 
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

