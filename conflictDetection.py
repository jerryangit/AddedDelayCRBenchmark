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
            "conflictZones": conflictZones,
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
    def detect(self,egoX,worldX,msg): 
        # slot is a list with [0], being the start of the slot and [1] being the end of the slot
        actor_t = msg.content.get("timeSlot")
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
                if s > self.sArrival + 0.5 and wr[0].transform.location.distance(worldX.inter_location)>=worldX.inter_bounds:
                    self.sExit = s
                    self.sPath.append([s,3,"exit"])
                else:
                    self.sPath.append([s,2,"cross"])
            else:
                self.sPath.append([s,4,"post"])
        return self.sPath
    def setup(self,egoX,worldX):
        self.sPathCalc(egoX,worldX)
        self.predictTimes(egoX,worldX)
#TODO improve TCL, now can miss some parts due to detection centered on vehicle
class gridCell:
    def __init__(self,center,resolution=4,size=16,error=0):
        self.center = center            # Carla.Location of intersection center
        self.resolution = resolution    # Ammount of cells per row/column
        self.size = size                # Distance in m between edges of grid        
        self.error = error
        self.calcGrid()
        self.arr = [0,0]
        self.ext = [0,0]

    def detect(self,egoX,worldX,msg): 
        actor_t = msg.content.get("timeSlot")
        actor_TCL = msg.content.get("TCL")
        ego_t = self.predictTimes(egoX,worldX)
        if ego_t[1] > actor_t[0] + self.error and ego_t[0] < actor_t[1] - self.error:
            TIC = cap(self.TCL,actor_TCL)
            if TIC != ():
                return [1,TIC]
        elif actor_t[1] > ego_t[0] + self.error and actor_t[0] < ego_t[1] - self.error:
            TIC = cap(self.TCL,actor_TCL)
            if TIC != ():
                return [1,TIC]
        return [0,()]

    def calcGrid(self):
        self.x_list = []
        self.y_list = []
        x0 = self.center.x - self.size/2
        y0 = self.center.y - self.size/2
        self.dx = self.size/self.resolution
        self.dy = self.size/self.resolution
        for i_x in range(self.resolution+1):
            self.x_list.append(x0+self.dx*i_x)
        for i_y in range(self.resolution+1):
            self.y_list.append(y0+self.dy*i_y)

    def sPathCalc(self,egoX,worldX):
        # TODO account for entire vehicle not just center
        self.sArrival = np.inf
        self.sExit = np.inf
        self.sPath = []
        self.TCL = []
        self.sTCL = {}

        s = 0
        s0 = egoX.route[0][0].transform.location
        for wr in egoX.route:
            s = wr[0].transform.location.distance(s0) + s
            s0 = wr[0].transform.location
            wr_x = wr[0].transform.location.x
            wr_y = wr[0].transform.location.y
            wr_x0 = wr_x - self.x_list[0]
            wr_y0 = wr_y - self.y_list[0]
            if self.sArrival == np.inf:
                if self.x_list[0] < wr_x and wr_x < self.x_list[-1] and self.y_list[0] < wr_y and wr_y < self.y_list[-1]:
                    self.sArrival = s
                    self.sPath.append([s,1,"enter"])
                    row = int(np.floor(wr_y0/self.dy))
                    col = int(np.floor(wr_x0/self.dx))
                    self.TCL.append((row,col))
                else:
                    self.sPath.append([s,0,"pre"])
            elif self.sExit == np.inf:
                if self.x_list[0]> wr_x or wr_x > self.x_list[-1] or self.y_list[0] > wr_y or wr_y > self.y_list[-1] :
                    self.sExit = s
                    self.sPath.append([s,3,"exit"])
                else:
                    row = int(np.floor(wr_y0/self.dy))
                    col = int(np.floor(wr_x0/self.dx))
                    if (row,col) != self.TCL[-1]: 
                        self.TCL.append((row,col))
                        self.sTCL[(row,col)] = s
                    self.sPath.append([s,2,"cross"])
            else:
                self.sPath.append([s,4,"post"])
    
    def predictTimes(self,egoX,worldX):
        if egoX.cr.cd.arr[0] != 1:
            self.arrivalTime = ( (self.sArrival-egoX.sTraversed) / egoX.velRef ) + worldX.tick.timestamp.elapsed_seconds
        else: 
            self.arrivalTime = egoX.cr.cd.arr[1]
        if egoX.cr.cd.ext[0] != 1:
            self.exitTime = ( (self.sExit-egoX.sTraversed) / egoX.velRef ) + worldX.tick.timestamp.elapsed_seconds
        else: 
            self.exitTime = egoX.cr.cd.ext[1]
        return (self.arrivalTime,self.exitTime)

    def cellTimes(self,TIC,egoX):
        a_max = 12
        arrDist = (self.sTCL[TIC]-self.sArrival)
        arrivalTime = self.arrivalTime + arrDist/egoX.velRef
        exitTime = arrivalTime + np.sqrt(2 * a_max * self.dx)/(2 * a_max)
        return [arrivalTime,exitTime]

    def TIC2Loc(self,TIC):
        x = self.x_list[TIC[1]]+0.5*self.dx
        y = self.y_list[TIC[0]]+0.5*self.dy
        z = self.center.z
        return carla.Location(x,y,z)

    def setup(self,egoX,worldX):
        self.sPathCalc(egoX,worldX)
        self.predictTimes(egoX,worldX)

class conflictZones:
    def __init__(self,center,resolution=4,size=16,error=0):
        self.center = center            # Carla.Location of intersection center
        self.resolution = resolution    # Ammount of cells per row/column
        self.size = size                # Distance in m between edges of grid        
        self.error = error
        self.calcGrid()

    def detect(self,egoX,worldX,msg): 
        actState = msg.content.get("state")
        actor_TCL = msg.content.get("TCL")
        actor_traj = msg.content.get("traj")
        TICL = caps(self.TCL,actor_TCL) # Trajectory intersecting cell list
        if len(TICL) != 0: # If the TICL is nonempty
            sptBool = 1
            if self.tempAdvantage(TICL,actor_traj,egoX.state,actState) == 1:
                tmpBool = 1
            else:
                tmpBool = 0
        else:
            sptBool = 0
            tmpBool = 0
        return (sptBool,tmpBool,TICL)

    def TICL(self,actor_TCL):
        TICL = caps(self.TCL,actor_TCL)
        return TICL

    def tempAdvantage(self,TICL,actorTraj,egoState,actState):
        # If Ego is crossing and Actor is first in lane
        if egoState == "I" and actState == "FIL":
            for conCell in TICL:
                egoT = self.traj.get(conCell)
                actorT = actorTraj.get(conCell)
                # If Ego enters a conflict zone before Actor leaves => Ego >TempAdv Actor 
                if egoT[0] < actorT[1]:
                    return 1
        # If Ego is first in line and Actor is in the intersection
        elif egoState == "FIL" and actState == "I":
            for conCell in TICL:
                egoT = self.traj.get(conCell)
                actorT = actorTraj.get(conCell)
                # If Ego leaves any conflict zone after Actor enters => Actor >TempAdv Ego 
                if egoT[1] > actorT[0]:
                    return 0
            # If Ego leaves all conflict zones before Actor enters => Ego >TempAdv Actor 
            return 1
        # If Ego and Actor have the same state, being either FIL or I
        elif egoState == actState and ( egoState == "FIL" or egoState == "I") :
            for conCell in TICL:
                egoT = self.traj.get(conCell)
                actorT = actorTraj.get(conCell)
                # If Ego enters any conflict zone before Actor enters => Ego >TempAdv Actor 
                if egoT[0] < actorT[0]:
                    return 1
        else:
            return 0

    def calcGrid(self):
        self.x_list = []        # List of grid lines x values
        self.y_list = []        # List of grid lines y values
        x0 = self.center.x - self.size/2
        y0 = self.center.y - self.size/2
        self.dx = self.size/self.resolution
        self.dy = self.size/self.resolution
        for i_x in range(self.resolution+1):
            self.x_list.append(x0+self.dx*i_x)
        for i_y in range(self.resolution+1):
            self.y_list.append(y0+self.dy*i_y)

    def sPathCalc(self,egoX,worldX):
        #* Calculate the path displacement and at which points it enters which areas
        self.sPath = []
        self.TCL = []
        self.sTCL = {} # sTCL.get(cell)[0] = sIn, sTCL.get(cell)[1] = sOut
        exitQueue = []
        s = 0
        s0 = egoX.route[0][0].transform.location
        # Loop through the waypoints of the entire route
        for wr in egoX.route:
            detectList = []
            s = wr[0].transform.location.distance(s0) + s               # Current displacement (prev disp + distance from previous waypoint)
            s0 = wr[0].transform.location                               # Current location
            wr_x = wr[0].transform.location.x                           # Current x value
            wr_y = wr[0].transform.location.y                           # Current y value
            wr_x0 = wr_x - self.x_list[0]                               # Current x value - first cell x
            wr_y0 = wr_y - self.y_list[0]                               # Current y value - first cell y
            
            # Update displacement of the center of the vehicle to self.sPath
            self.sPath.append(s)

            # Generate samples for vehicle at current waypoint
            smpArr = generateSamples(egoX.ego,5,s0)
            # For each sample check the area it is in
            for smp in smpArr:
                # x = smp[0], y = smp[1]
                cell = self.xy2Cell(smp[0],smp[1])
                # Save cell in detectList if sample is in the grid
                if cell not in detectList and self.inGrid(cell):
                    detectList.append(cell)
                    # Add cell to TCL and save its entry s if new
                    if cell not in self.TCL:
                        exitQueue.append(cell)
                        self.TCL.append(cell)
                        self.sTCL[cell] = (s,None)    

            # Go through cells in exitQueue and determine if they are still occupied or not                                        
            for cell in exitQueue:
                if cell not in detectList:
                    self.sTCL[cell] = (self.sTCL.get(cell)[0],s)
                    exitQueue.remove(cell)

    def predictTimes(self,egoX,worldX):
        # TODO is this the best way to do this?
        #* Predict times for a certain traj
        traj = {}
        for cell in self.TCL:
            # time in for a given cell computed by (distance remaining)/(reference velocity) + current time
            (sIn,sOut) = self.sTCL.get(cell)
            tIn = (sIn-egoX.sTraversed)/egoX.velRef + worldX.tick.timestamp.elapsed_seconds 
            tOut = (sOut-egoX.sTraversed)/egoX.velRef + worldX.tick.timestamp.elapsed_seconds + self.error
            traj[cell] = (tIn,tOut)
        self.traj = traj
        # self.intersectionTime
        return self.traj

    def inGrid(self,cell):
        if cell[0] >= 0 and cell[0] <= self.resolution:
            if cell[1] >= 0 and cell[1] <= self.resolution: 
                return 1
        return 0

    def loc2Cell(self,location):
        #* Returns the cell (row,col) for a Carla.Location object
        row = int(np.floor((location.y - self.y_list[0]))/self.dy)
        col = int(np.floor((location.x - self.x_list[0]))/self.dx)
        cell = (row,col)
        return cell

    def xy2Cell(self,x,y):
        #* Returns the cell (row,col) for a Carla.Location object
        row = int(np.floor((y - self.y_list[0]))/self.dy)
        col = int(np.floor((x - self.x_list[0]))/self.dx)
        cell = (row,col)
        return cell

    def cell2Loc(self,cell):
        #* Returns the Carla.Location object at the center of a cell (row,col)
        x = self.x_list[cell[1]]+0.5*self.dx
        y = self.y_list[cell[0]]+0.5*self.dy
        z = self.center.z
        return carla.Location(x,y,z)

    def setup(self,egoX,worldX):
        #* Ran once to setup the object
        self.sPathCalc(egoX,worldX)
        self.predictTimes(egoX,worldX)


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

def generateSamples(actor,sampleN=5,location=None):
    # TODO integrate some of these into helper functions
    # Determine lenght of bbox in x and y directions
    xLength = actor.bounding_box.extent.x * 2
    yLength = actor.bounding_box.extent.y * 2
    # Def amount of sample along x and y edges, rounded to int
    xSamples = int( round( sampleN * ( xLength / ( xLength + yLength ) ) ) )
    ySamples = sampleN - xSamples
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
        # If no location is provided use actor location
    if location == None:
        smpArr = smpArr + [ actor.get_location().x , actor.get_location().y ]
        # If location is provided use given location
    else:
        smpArr = smpArr + [ location.x , location.y ]
    return smpArr


def cap(A,B):
    # Returns first common element search depth first through x with y
    cap = ()
    for x in A:
        for y in B:
            if x == y:
                return x
    return cap


def caps(A,B):
    # Returns all common element search depth first through x with y
    cap = []
    for x in A:
        for y in B:
            if x == y:
                cap.append(x)
    return cap