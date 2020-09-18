#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from cvxopt import matrix,solvers
import actorHelper as ah
import conflictDetection as cd
from mpc import qpMPC
import glob
import os
import sys
import csv
import datetime
# try:
#     sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass

import carla
import random
import numpy as np

import matplotlib.pyplot as plt
def actorControl(arg,para=[]):
    cases = {
        "TEPControl": TEPControl,
        "MPIPControl": MPIPControl,
        "DCRControl": DCRControl,
        "OAMPC": OAMPC,
    }
    fnc = cases.get(arg)
    if isinstance(para, list):
        return fnc(*para)
    else:
        return fnc(para)

class TEPControl:
    def __init__(self):
        self.vPIDStates = (0,0,0)
        self.thetaPIDStates = (0,0,0)
        self.cd_obj = cd.conflictDetection("coneDetect",[9,0.135*np.pi,5]).obj
    def control(self,egoX,worldX):
        ego = egoX.ego
        if egoX.location == carla.Location(0,0,0):
            return "NAN"
        velDes = egoX.velRef

        if egoX.sTraversed > 20 and worldX.tick.elapsed_seconds < 10:
            ego.apply_control(carla.VehicleControl(throttle=0, steer=0,brake = 1,manual_gear_shift=True,gear=1))
            return "TESTING"

        if len(egoX.cr.wait) > 0:
            for cell in egoX.cr.wait.values():
                if cell in egoX.cr.cd.sTCL.keys():
                    sIn = egoX.cr.cd.sTCL.get(cell)[0]
                    dist = sIn - egoX.sTraversed
                    if dist < 0:
                        velDes = 0
                        print(egoX.id," crossed line by:", np.abs(dist),"m, adjust breaking please")
                    elif dist < 0.275:
                        velDes = 0
                    elif dist < 10:
                        velDes = min((dist/1.5 - egoX.velNorm/(egoX.aMin)*0.15,velDes))
                        if velDes < 0:
                            velDes = 0

        if egoX.state == "ENTER":
            for actor in worldX.actorDict.actor_list:
                if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 10 : 
                    cd_bool,smpList = self.cd_obj.detect(ego,actor,1)
                    if cd_bool == 1:
                        if smpList[0] < 5:
                            velDes = 0
                        else:
                            velDes = min(smpList[0]/2,velDes)

        (u_v,self.vPIDStates) = velocityPID(egoX,self.vPIDStates,velDes)
        (u_theta,self.thetaPIDStates)  = anglePID(egoX,worldX,self.thetaPIDStates)

        if u_v >= 0:
            ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0,manual_gear_shift=True,gear=1))
        elif u_v < 0:
            ego.apply_control(carla.VehicleControl(throttle=0, steer=u_theta,brake = - u_v,manual_gear_shift=True,gear=1))
        u_v = np.clip(u_v,-1,1)


        state = egoX.state

        if egoX.state == "NAN":
            state = "ENTER"
        elif egoX.state == "ENTER" and egoX.sTraversed > egoX.cr.cd.sArrival-1:
            if egoX.cr.cd.arr[0] != 1:
                egoX.cr.cd.arr = [1,egoX.cr.cd.arrivalTime]
            state = "CROSS"
        elif egoX.state == "CROSS" and egoX.sTraversed > egoX.cr.cd.sExit:
            if egoX.cr.cd.ext[0] != 1:
                egoX.cr.cd.ext = [1,egoX.cr.cd.exitTime]
            state = "EXIT"
        egoX.discreteState(state)
        return state

class MPIPControl:
    def __init__(self):
        self.vPIDStates = (0,0,0)
        self.thetaPIDStates = (0,0,0)
        self.cd_obj = cd.conflictDetection("coneDetect",[6.5,0.135*np.pi,5]).obj

    def control(self,egoX,worldX):
        ego = egoX.ego
        if egoX.location == carla.Location(0,0,0):
            return "NAN"
        
        velDes = egoX.velRef

        if len(egoX.cr.wait) > 0:
            for cell in egoX.cr.wait.values():
                if cell in egoX.cr.cd.sTCL.keys():
                    sIn = egoX.cr.cd.sTCL.get(cell)[0]
                    dist = sIn - egoX.sTraversed
                    if dist < 0:
                        velDes = 0
                        print(egoX.id," crossed line by:", np.abs(dist),"m, adjust breaking please")
                    elif dist < 0.275:
                        velDes = 0
                    elif dist < 10:
                        velDes = min((dist/1.5 - egoX.velNorm/(egoX.aMin)*0.15,velDes))
                        if velDes < 0:
                            velDes = 0


        # if egoX._spwnNr == 2:
        #     plt.figure(12)
        #     plt.title(str(egoX.id)+' wait:'+str(egoX.cr.wait))
        #     plt.plot(worldX.tick.elapsed_seconds,velDes,'gx')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.velNorm,'bx')
        #     if len(egoX.cr.wait) > 0:
        #         plt.plot(worldX.tick.elapsed_seconds,0,'rx')
        #     plt.show(block=False)
        #     plt.pause(0.001)

        #     plt.figure(13)
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.cr.cd.sArrival,'gx')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.sTraversed,'bx')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.cr.cd.arrivalTime,'cx')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.cr.cd.exitTime,'c.')
        #     for cell in egoX.cr.cd.traj.keys():
        #         times = egoX.cr.cd.traj.get(cell)
        #         clr = (cell[0]/3,cell[1]/3,0)
        #         plt.plot((worldX.tick.elapsed_seconds,worldX.tick.elapsed_seconds),(times[0],times[1]),'--',color = clr)
        #     plt.show(block=False)
        #     plt.pause(0.01567)

        if egoX.state == "ENTER":
            for actor in worldX.actorDict.actor_list:
                if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 10 : 
                    cd_bool,smpList = self.cd_obj.detect(ego,actor,1)
                    if cd_bool == 1:
                        if smpList[0] < 5:
                            velDes = 0
                        else:
                            velDes = min((smpList[0]/2,velDes))

        (u_v,self.vPIDStates) = velocityPID(egoX,self.vPIDStates,velDes)
        (u_theta,self.thetaPIDStates)  = anglePID(egoX,worldX,self.thetaPIDStates)
        u_v = np.clip(u_v,-1,1)

        if u_v >= 0:
            ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0,manual_gear_shift=True,gear=1))
        elif u_v < 0:
            ego.apply_control(carla.VehicleControl(throttle=0, steer=u_theta,brake = - u_v,manual_gear_shift=True,gear=1))



        state = egoX.state
        if egoX.state == "NAN":
            state = "ENTER"
        elif egoX.state == "ENTER" and egoX.sTraversed > egoX.cr.cd.sArrival - 1:
            if egoX.cr.cd.arr[0] != 1:
                egoX.cr.cd.arr = [1,egoX.cr.cd.arrivalTime]
            state = "CROSS"
        elif egoX.state == "CROSS" and egoX.sTraversed > egoX.cr.cd.sExit:
            if egoX.cr.cd.ext[0] != 1:
                egoX.cr.cd.ext = [1,egoX.cr.cd.exitTime]
            state = "EXIT"
        egoX.discreteState(state)
        return state

class DCRControl:
    def __init__(self):
        self.vPIDStates = (0,0,0)
        self.thetaPIDStates = (0,0,0)
        self.cd_obj = cd.conflictDetection("coneDetect",[3.5,0.1*np.pi,5]).obj

    def control(self,egoX,worldX):
        ego = egoX.ego
        if egoX.location == carla.Location(0,0,0):
            return "NAN"

        #* MPC velocity control:
        x0 = np.array([[egoX.sTraversed],[egoX.velNorm]])
        # cell = (sInCell,TinCell)
        # From egoX T received as time from current timestep
        CellList = [] 
        for cell in egoX.cr.cd.traj:
            # For each cell in traj get its sIn and tIn
            # TODO add constraint at last timestep to if first cell is outside of its horizon
            CellList.append((egoX.cr.cd.sTCL.get(cell)[0],egoX.cr.cd.traj.get(cell)[0]-worldX.tick.timestamp.elapsed_seconds))
        # TODO move N to be an input

        N = 30

        velDes = egoX.velRef
        if egoX.state == "IL":
            for actor in worldX.actorDict.actor_list:
                if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 10 : 
                    cd_bool,smpList = self.cd_obj.detect(ego,actor,1)
                    if cd_bool == 1:
                        if smpList[0] < 4:
                            velDes = 0

        if egoX.state != "OL":
            (sol,u0) = qpMPC(x0,egoX.dt*4,egoX.velRef,CellList,egoX.id,N)
            egoX.cr.cd.MPCFeedback(sol,worldX.tick.timestamp.elapsed_seconds,egoX.dt*4,N,2,egoX.velRef,egoX.id,egoX.sTraversed) # dt is multiplied in order to increase finite horizon time range
            velDes = min(egoX.velNorm+u0[0]*1.3,velDes)
        else: 
            velDes = egoX.velRef

        (u_v,self.vPIDStates) = velocityPID(egoX,self.vPIDStates,velDes)
        #* PID Control

        (u_theta,self.thetaPIDStates)  = anglePID(egoX,worldX,self.thetaPIDStates)
        u_v = np.clip(u_v,-1,1)


        if u_v >= 0:
            ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0,manual_gear_shift=True,gear=1))
        elif u_v < 0:
            ego.apply_control(carla.VehicleControl(throttle=0, steer=u_theta,brake = - u_v*1.2,manual_gear_shift=True,gear=1))


        state = egoX.state
        if egoX.state == "NAN":
            state = "IL"
        elif egoX.state == "IL" and egoX.info.get("idFront") == None:
            queue = [(egoX.id,egoX.location.distance(worldX.inter_location))]
            for msg in worldX.msg.inbox[egoX.id]:
                # If the sender is not in the intersection or leaving and on the same road_id:
                # if msg.content.get("state") not in ["I","OL"] and msg.content.get("wp") != [] and msg.content.get("wp").road_id == egoX.waypoint.road_id:
                if msg.content.get("state") not in ["I","OL"] and msg.content.get("spwnid") == egoX.spwnid:
                    queue.append((msg.idSend,worldX.actorDict.dict.get(msg.idSend).location.distance(worldX.inter_location)))
            queue.sort(key=lambda x: x[1])
            if queue[0][0] == egoX.id:
                state = "FIL"
            else: 
                for pair in enumerate(queue):
                    if pair[1][0] == egoX.id:
                        egoX.infoSet("idFront",queue[pair[0] - 1][0])
        elif egoX.state == "FIL" and egoX.sTraversed > egoX.cr.cd.sTCL[egoX.cr.cd.TCL[0]][0]+0.25:
            state = "I"
        elif egoX.state == "I" and egoX.sTraversed > egoX.cr.cd.sTCL[egoX.cr.cd.TCL[-1]][1]+0.25:
            state = "OL"
        egoX.discreteState(state)
        return state

class OAMPC:
    def __init__(self):
        self.aPIDStates = (0,0,0,0)
        self.u_a0 = 0
        self.u_delta0 = 0

    def control(self,egoX,worldX):
        # Input acceleration
        a = egoX.cr.ctrl[0]
        # Input steering angle
        u_delta = egoX.cr.ctrl[1]/((4*np.pi)/9)

        theta = ((-egoX.ego.get_transform().rotation.yaw*np.pi)/180)%(2*np.pi)
        R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        egoX.accLoc =  R@np.array([egoX.ego.get_acceleration().x,egoX.ego.get_acceleration().y])
        egoX.velLoc =  R@np.array([egoX.ego.get_velocity().x,egoX.ego.get_velocity().y])



        (u_a,self.aPIDStates) = accPID(egoX,self.aPIDStates,a)
        u_a = 0.5*u_a+0.5*self.u_a0
        u_delta_ratio = 0.5
        u_delta = u_delta_ratio*u_delta+(1-u_delta_ratio)*self.u_delta0/((4*np.pi)/9)

        u_a = np.clip(u_a,-1,1)
        u_delta = np.clip(u_delta,-1,1)
        if u_a > 0.05:
            egoX.ego.apply_control(carla.VehicleControl(throttle=u_a, steer=u_delta,brake = 0,manual_gear_shift=True,gear=1))
        elif u_a <= 0.05 and u_a > -0.25:
            egoX.ego.apply_control(carla.VehicleControl(throttle=0, steer=u_delta,brake = 0,manual_gear_shift=True,gear=1))            
        elif egoX.velLoc[0] <= 0.25:
            egoX.ego.apply_control(carla.VehicleControl(throttle=-u_a*1.5, steer=u_delta,brake = 0,manual_gear_shift=True,gear=-1))
        else:
            egoX.ego.apply_control(carla.VehicleControl(throttle=0, steer=u_delta,brake = -u_a,manual_gear_shift=True,gear=1))
        self.u_a0 = u_a
        self.u_delta0 = u_delta*((4*np.pi)/9)
        # if egoX._spwnNr == 0:
        #     # plt.figure(12)
        #     # plt.title(str(egoX.id)+'Acceleration')
        #     # plt.ylim(-5,10)            
        #     # plt.plot(worldX.tick.elapsed_seconds,a,'gx')
        #     # plt.plot(worldX.tick.elapsed_seconds,egoX.accLoc[0],'bx')

        #     plt.figure(13)
        #     plt.title(str(egoX.id)+'Acceleration')
        #     plt.ylim(-10,10)            
        #     plt.plot(worldX.tick.elapsed_seconds,a,'gx')
        #     # plt.plot(worldX.tick.elapsed_seconds,self.aPIDStates[3],'bx')
        #     plt.plot(worldX.tick.elapsed_seconds,u_a,'rx')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.accLoc[0],'bx')

        #     # plt.plot(worldX.tick.elapsed_seconds,self.aPIDStates[0],'gx')

        #     plt.figure(14)
        #     plt.title(str(egoX.id)+'Velocity')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.velNorm,'gx')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.velRef,'bx')
            
        #     plt.figure(14)
        #     plt.title(str(egoX.id)+'Steering Angle')
        #     plt.plot(worldX.tick.elapsed_seconds,egoX.cr.ctrl[1],'gx')
        #     plt.plot(worldX.tick.elapsed_seconds,u_delta,'bx')


        #     if worldX.tick.elapsed_seconds > 10:
        #         plt.show()
        #         # plt.show(block=False)
        #         # plt.pause(0.001)

        #     # plt.show(block=False)
        #     # plt.pause(0.001)


def accPID(egoX,states,aRef):
    kp = 0.2
    kd = 0.0
    ki = 0.1
    #* >
    a = 1*egoX.accLoc[0] + 0*states[3]
    
    #* PID states <
    error = aRef - a
    integral = states[1] + error * egoX.dt
    derivative = (error - states[0]) / egoX.dt
    #* >
    u_a = kp*(error) + ki*(integral) + kd*(derivative) 
    v = np.linalg.norm([egoX.ego.get_velocity().x,egoX.ego.get_velocity().y])
    if  v > 1:
        u_a += 0.115+0.0936*v+-0.00345*v**2 # quadractic approximation of throttle to overcome friction
    else:
        u_a += 0.1+0.11*v # quadractic approximation of throttle to overcome friction
        
    states = (error,integral,derivative,a)
    return (u_a,states)


def velocityPID(egoX,states,velDes=None):
    #* PID Gains < 
    kp = 0.9
    kd = 0.1
    ki = 0
    #* >
    v = egoX.velNorm
    #* PID states <
    if velDes != None:
        error = velDes - v
    else:
        error = egoX.velRef - v
    integral = states[1] + error * egoX.dt
    derivative = (error - states[0]) / egoX.dt
    #* >
    u_v = kp*(error) + ki*(integral) + kd*(derivative) 
    if u_v < 0: 
        u_v = u_v * 1.1
    if v > 1:
        u_v += 0.115+0.0936*v+-0.00345*v**2 # quadractic approximation of throttle to overcome friction
    else:
        u_v += 0.1+0.11*v # quadractic approximation of throttle to overcome friction
        
    states = (error,integral,derivative)
    return (u_v,states)

def anglePID(egoX,worldX,states):
    #* PID Gains
    kp = 1.75
    kd = 0.02
    ki = 0.02
    #* 
    
    # Make a list of the current + next 9 waypoints and sort it
    dist = []
    for i in range(10):
        if egoX.routeIndex+i < len(egoX.route):
            dist.append([egoX.location.distance(egoX.route[egoX.routeIndex+i][0].transform.location),i])
    # the closest waypoint is deemed to be the current index.
    egoX.routeIndex = egoX.routeIndex+sorted(dist)[0][1]
    
    # Angle of the current waypoint
    theta_way = wrap((np.pi*egoX.route[egoX.routeIndex][0].transform.rotation.yaw)/180)
    # Current Angle
    theta_ego = wrap((np.pi*egoX.rotation.yaw)/180)
    # TODO Steer angle towards waypoint doesn't work
    # # Angle towards next waypoint to correct deviation from path
    # try:
    #     dy = egoX.route[egoX.routeIndex+1][0].transform.location.y - egoX.location.y
    #     dx = egoX.route[egoX.routeIndex+1][0].transform.location.x - egoX.location.x
    #     theta_dir = np.arctan2(dy,dx)
    # except:
    #     theta_dir = theta_way
    #* PID states <
    error = wrap(theta_way-theta_ego)
    # print(round(theta_dir,3),"|",round(theta_ego,3),"|",round(wrap(theta_dir-theta_ego),5),"|",np.sign(dy),"|",np.sign(dx))
    integral = states[1] + error * egoX.dt
    derivative = (error - states[0]) / egoX.dt
    #* >
    u_theta = kp*(error) + ki*(integral) + kd*(derivative)
    egoX.waypoint = worldX.map.get_waypoint(egoX.location,1)
    # print("ID: ",egoX.id,"|U_Theta: ", u_theta)
    return (u_theta,states)


def wrap(angle):
    while np.abs(angle) > np.pi:
        if angle > np.pi:
            angle = angle - 2 * np.pi
        elif angle < -np.pi:
            angle = angle + 2 * np.pi
    return angle