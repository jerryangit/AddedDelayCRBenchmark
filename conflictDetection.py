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
        ego_t = (self.arrivalTime,self.exitTime)
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
        return (self.arrivalTime,self.exitTime)
    
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
        exitQueue = []

        for wr in egoX.route:
            detectList = []
            s = wr[0].transform.location.distance(s0) + s               # Current displacement (prev disp + distance from previous waypoint)
            s0 = wr[0].transform.location                               # Current location

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
                        if len(self.TCL) == 1:
                            self.sArrival = s

            # Go through cells in exitQueue and determine if they are still occupied or not                                        
            for cell in exitQueue:
                if cell not in detectList:
                    self.sTCL[cell] = (self.sTCL.get(cell)[0],s)
                    exitQueue.remove(cell)
                    if len(exitQueue) == 0:
                        self.sExit = s

    def updateTCL(self,sTraversed):
        for cell in self.TCL:
            # If sTraversed is larger than the exit displacement, remove it from the TCL
            if sTraversed > self.sTCL.get(cell)[1] + 0.10:
                self.TCL.remove(cell)
                del self.sTCL[cell]
    
    def predictTimes(self,egoX,worldX):
        # TODO is this the best way to do this?
        #* Predict times for a certain traj
        traj = {}
        if egoX.cr.cd.arr[0] != 1:
            self.arrivalTime = ( (self.sArrival-egoX.sTraversed) / egoX.velRef ) + worldX.tick.timestamp.elapsed_seconds
        else: 
            self.arrivalTime = egoX.cr.cd.arr[1]
        if egoX.cr.cd.ext[0] != 1:
            self.exitTime = ( (self.sExit-egoX.sTraversed) / egoX.velRef ) + worldX.tick.timestamp.elapsed_seconds
        else: 
            self.exitTime = egoX.cr.cd.ext[1]
        for cell in self.TCL:
            # time in for a given cell computed by (distance remaining)/(reference velocity) + current time
            (sIn,sOut) = self.sTCL.get(cell)
            tIn = (sIn-egoX.sTraversed)/egoX.velRef + worldX.tick.timestamp.elapsed_seconds 
            tOut = (sOut-egoX.sTraversed)/egoX.velRef + worldX.tick.timestamp.elapsed_seconds + self.error
            traj[cell] = (tIn,tOut)
        self.traj = traj
        return (self.arrivalTime,self.exitTime)

    def inGrid(self,cell):
        if cell[0] >= 0 and cell[0] <= self.resolution-1:
            if cell[1] >= 0 and cell[1] <= self.resolution-1: 
                return 1
        return 0

    def xy2Cell(self,x,y):
        #* Returns the cell (row,col) for a Carla.Location object
        row = int(np.floor((y - self.y_list[0])/self.dy))
        col = int(np.floor((x - self.x_list[0])/self.dx))
        cell = (row,col)
        return cell

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

    def sptDetect(self,egoX,worldX,msg): 
        actor_TCL = msg.content.get("TCL")
        TICL = caps(self.TCL,actor_TCL) # Trajectory intersecting cell list
        if len(TICL) != 0: # If the TICL is nonempty
            return (1,TICL)
        else:
            return (0,TICL)

    def TICL(self,actor_TCL):
        TICL = caps(self.TCL,actor_TCL)
        return TICL


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

    def MPCFeedback(self,sol,t0,dt,N,states,velRef,id,sTraversed):
        s_1k = sol['x'][0]
        for k in range(N):
            s_k = sol['x'][states*k]
            for cell in self.traj.keys():
                (sIn,sOut) = self.sTCL.get(cell) 
                # If the sIn is bigger than previous s but smaller than current, we use tIn from previous for tIn
                if s_1k < sIn and sIn < s_k:
                    # tIn is predicted some ti
                    # mesteps ahead in order to prevent it affecting the MPC as a constraint
                    tIn = t0+(k-1)*dt - 0.5 
                    delay = tIn - self.traj.get(cell)[0]
                    # If the vehicle can enter earlier than estimated.
                    if delay < 0:
                        self.traj[cell] = (tIn,self.traj.get(cell)[1]+delay)
                # If the sOut is bigger than previous s but smaller than current, we use tOut for current displacement
                if s_1k < sOut and sOut < s_k:
                    # if t0+k*dt < self.traj.get(cell)[0]:
                    #     print("Error, out time lower than entry time")
                    self.traj[cell] = (self.traj.get(cell)[0],t0+k*dt)
                if self.traj[cell][1] < self.traj[cell][0]:
                    print("tIn<tOut")    
            s_1k = s_k
        s_max = s_1k
        for cell in self.traj.keys():
            (sIn,sOut) = self.sTCL.get(cell)
            # If difference between entrance and maximum reached distance within finite horizon is larger than 1m
            if sIn - s_max > 1:
                # self.traj[cell] = ((sIn-s_max)/(0.5*sol['x'][N*states-1]+0.5*velRef) + t0+N*dt,(sOut-s_max)/(0.5*sol['x'][N*states-1]+0.5*velRef) + t0+N*dt)
                tIn = (sIn-s_max)/(velRef) + t0+N*dt 
                delay = tIn - self.traj.get(cell)[0]
                self.traj[cell] = (tIn,self.traj.get(cell)[1]+delay)
            if self.traj[cell][1] < self.traj[cell][0]:
                print("tIn<tOut")    
        
        if 1==0:
            import matplotlib.pyplot as plt
            plt.figure(1)
            plt.clf()
            for cell in self.traj.keys():
                (sIn,sOut) = self.sTCL.get(cell) 
                (tIn,tOut) = self.traj.get(cell)
                plt.plot(tIn,sIn,'b>')
                plt.plot(tOut,sOut,'r<')
                plt.plot([tIn,tOut],[sIn,sOut],'--')
            splot = []
            tplot = []
            for i in range(N):
                splot.append(sol['x'][i*2])
                tplot.append(t0+i*dt)
            plt.plot(tplot,splot,'g')
            plt.plot(t0,sTraversed,'gx')
            plt.vlines(t0,0,60)
            plt.vlines(t0+N*dt,0,60)
            plt.xlim(left=0)
            plt.xlim(right=40)
            plt.title(str(id))                
            plt.show(block=False)
            plt.pause(0.01)

    def inGrid(self,cell):
        if cell[0] >= 0 and cell[0] <= self.resolution-1:
            if cell[1] >= 0 and cell[1] <= self.resolution-1: 
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
        row = int(np.floor((y - self.y_list[0])/self.dy))
        col = int(np.floor((x - self.x_list[0])/self.dx))
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

    def detect(self,ego,actor,mode=0):
        smpArr = generateSamples(actor,self.actorSamples)
        if mode == 0: # return on first sample within range
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
        elif mode == 1: # return closes sample
            smpList = []
            for i in range(smpArr.shape[0]):
                # relative vector from ego vehicle to sample
                vec = smpArr[i]- [ ego.get_location().x , ego.get_location().y ]
                r = np.linalg.norm( vec )
                # Note: Coordinate frame carla has left handed coordinate frame!!
                phi0 = (ego.get_transform().rotation.yaw * np.pi)/ 180 
                phi = np.arctan2( vec[1] , vec[0] ) - phi0
                if r < self.radius and np.absolute( phi ) < 0.5 * self.angle:
                    smpList.append((r,phi))
            if len(smpList) > 0:
                return (1,sorted(smpList,key=lambda x: x[0])[0])
            return (0,[None])

def generateSamples(actor,sampleN=5,location=None):
    # TODO integrate some of these into helper functions
    # Determine lenght of bbox in x and y directions
    xLength = actor.bounding_box.extent.x * 1.8
    yLength = actor.bounding_box.extent.y * 1.8
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

    # import matplotlib.pyplot as plt
    # plt.figure(5)
    # for sample in smpArr:
    #     plt.plot(sample[0],sample[1],'rx')
    # plt.plot(actor.get_location().x,actor.get_location().y,'go')
    # plt.show()

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