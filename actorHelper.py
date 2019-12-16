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
    def coneDetect(self,actor):
        smpArr = self.genSamples(actor)
        for i in range(smpArr.shape[0]):
            # relative vector from ego vehicle to sample
            vec = smpArr[i]- [ self.ego.get_location().x , self.ego.get_location().y ]
            r = np.linalg.norm( vec )
            # Note: Coordinate frame carla has left handed coordinate frame!!
            phi0 = (self.ego.get_transform().rotation.yaw * np.pi)/ 180 
            phi = np.arctan2( vec[1] , vec[0] ) - phi0
            ##############################################
            #
            # Bug in angle, check if accounted for orientation or something ,vehicle with -119,6 -37 detected collision with vehicle behind it at -113,7 -36.8 in east lane
            #
            ##############################################
            if r < self.radius and np.absolute( phi ) < 0.5 * self.angle:
                 return 1           
        return 0
    def genSamples(self,actor):
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




### Velocity based col detect 
# T_ij = F_j.location - F_i.location
# v_i =  self.get_velocity()
# v_j = actor.get_velocity()
# v_ij = v_i - v_j
# dT1_ij = T_ij-v_ij*rt
# rdiff = np.linalg.norm(np.array([dT1_ij.x,dT1_ij.y,dT1_ij.z]))
# if rdiff <= 5 and np.absolute((F_i.rotation.yaw/180)*np.pi - np.arctan2(T_ij.y,T_ij.x)) < 0.5 * np.pi:
    # self.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0))
# if rdiff <= 5 and np.absolute((F_i.rotation.yaw/180)*np.pi - np.arctan2(T_ij.y,T_ij.x)) > 0.5 * np.pi:
    # self.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0,brake = 0.0))
## Detect sample in cone