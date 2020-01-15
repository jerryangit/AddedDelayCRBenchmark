#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from cvxopt import matrix,solvers
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

def actorControl(arg,para=[]):
    cases = {
        "TEPControl": TEPControl,
        "MPIPControl": MPIPControl,
        "AMPIPControl": AMPIPControl,
        "DCRControl": DCRControl,
        "test": test,
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
    def control(self,egoX,worldX):
        # TODO Fix arrival time when near intersection, shouldn't increase as it is FCFS
        ego = egoX.ego
        if egoX.location == carla.Location(0,0,0):
            return "NAN"
        (u_v,self.vPIDStates) = velocityPID(egoX,self.vPIDStates)
        (u_theta,self.thetaPIDStates)  = anglePID(egoX,worldX,self.thetaPIDStates)
        ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0.0,manual_gear_shift=True,gear=1))
        cd_obj = cd.conflictDetection("coneDetect",[6.5,0.135*np.pi,5]).obj
        for actor in worldX.actorDict.actor_list:
            if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 12 : 
                if cd_obj.detect(ego,actor):
                    ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0,manual_gear_shift=True,gear=1))
        if len(egoX.cr.wait) > 0 and ego.get_location().distance(worldX.inter_location) <= worldX.inter_bounds + 3 :
            if egoX.cr.cd.arr[0] != 1:
                egoX.cr.cd.arr = [1,egoX.cr.cd.arrivalTime]
            ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0,manual_gear_shift=True,gear=1))        
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
        egoX.discreteState(state)
        return state

class MPIPControl:
    def __init__(self):
        self.vPIDStates = (0,0,0)
        self.thetaPIDStates = (0,0,0)
    def control(self,egoX,worldX):
        # TODO Fix arrival time when near intersection, shouldn't increase as it is FCFS
        ego = egoX.ego
        if egoX.location == carla.Location(0,0,0):
            return "NAN"
        (u_v,self.vPIDStates) = velocityPID(egoX,self.vPIDStates)
        (u_theta,self.thetaPIDStates)  = anglePID(egoX,worldX,self.thetaPIDStates)
        ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0.0,manual_gear_shift=True,gear=1))
        cd_obj = cd.conflictDetection("coneDetect",[6.5,0.135*np.pi,5]).obj
        if egoX.state == "ENTER":
            for actor in worldX.actorDict.actor_list:
                if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 12: 
                    if cd_obj.detect(ego,actor):
                        ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0,manual_gear_shift=True,gear=1))
        if len(egoX.cr.wait) > 0:
            for idSend in egoX.cr.wait:
                dist = ego.get_location().distance(egoX.cr.cd.TIC2Loc(egoX.cr.wait[idSend]))
                if dist<= egoX.cr.cd.dx + 0.15 + 1 * egoX.vel_norm:
                    u = 1/dist * egoX.vel_norm
                    ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = u,manual_gear_shift=True,gear=1))
        state = egoX.state
        if egoX.state == "NAN":
            state = "ENTER"
        elif egoX.state == "ENTER" and ego.get_location().distance(worldX.inter_location) <= worldX.inter_bounds+2.5:
            if egoX.cr.cd.arr[0] != 1:
                egoX.cr.cd.arr = [1,egoX.cr.cd.arrivalTime]
            state = "CROSS"
        elif egoX.state == "CROSS" and ego.get_location().distance(worldX.inter_location) >= worldX.inter_bounds+3:
            if egoX.cr.cd.ext[0] != 1:
                egoX.cr.cd.ext = [1,egoX.cr.cd.exitTime]
            state = "EXIT"
        egoX.discreteState(state)
        return state

class AMPIPControl:
    def __init__(self):
        self.vPIDStates = (0,0,0)
        self.thetaPIDStates = (0,0,0)

    def control(self,egoX,worldX):
        # TODO Fix arrival time when near intersection, shouldn't increase as it is FCFS
        ego = egoX.ego
        if egoX.location == carla.Location(0,0,0):
            return "NAN"
        (u_v,self.vPIDStates) = velocityPID(egoX,self.vPIDStates)
        (u_theta,self.thetaPIDStates)  = anglePID(egoX,worldX,self.thetaPIDStates)
        ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0.0,manual_gear_shift=True,gear=1))
        cd_obj = cd.conflictDetection("coneDetect",[6.5,0.135*np.pi,5]).obj
        if egoX.state == "ENTER":
            for actor in worldX.actorDict.actor_list:
                if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 12 : 
                    if cd_obj.detect(ego,actor):
                        ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0,manual_gear_shift=True,gear=1))
        if len(egoX.cr.wait) > 0:
            for idSend in egoX.cr.wait:
                dist = egoX.location.distance(egoX.cr.cd.TIC2Loc(egoX.cr.wait[idSend])) 
                if dist<= egoX.cr.cd.dx + 0.65 + 0.25 * egoX.vel_norm  :
                    u = 1/dist * egoX.vel_norm * 2
                    ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = u,manual_gear_shift=True,gear=1))
        state = egoX.state
        if egoX.state == "NAN":
            state = "ENTER"
        elif egoX.state == "ENTER" and ego.get_location().distance(worldX.inter_location) <= worldX.inter_bounds+2.5:
            if egoX.cr.cd.arr[0] != 1:
                egoX.cr.cd.arr = [1,egoX.cr.cd.arrivalTime]
            state = "CROSS"
        elif egoX.state == "CROSS" and ego.get_location().distance(worldX.inter_location) >= worldX.inter_bounds+3:
            if egoX.cr.cd.ext[0] != 1:
                egoX.cr.cd.ext = [1,egoX.cr.cd.exitTime]
            state = "EXIT"
        egoX.discreteState(state)
        return state

class DCRControl:
    def __init__(self):
        self.vPIDStates = (0,0,0)
        self.thetaPIDStates = (0,0,0)
    def control(self,egoX,worldX):
        ego = egoX.ego
        if egoX.location == carla.Location(0,0,0):
            return "NAN"
        if egoX.state == "NAN":
            self.nextCell = egoX.cr.cd.TCL[0]
        #* Time constraints
        self.currentCell = egoX.cr.cd.loc2Cell(egoX.location)
        if self.currentCell in egoX.cr.cd.TCL:
            cellIndex = egoX.cr.cd.TCL.index(self.currentCell)
            if cellIndex < len(egoX.cr.cd.TCL)-1:
                self.nextCell = egoX.cr.cd.TCL[cellIndex+1]
            else:
                self.nextCell = ()
        # if worldX.tick.elapsed_seconds + egoX.location.distance(egoX.cr.cd.cell2Loc(nextCell))/egoX.vel_norm < egoX.cr.cd.traj.get(nextCell)[0]:
        #     dist = egoX.location.distance(egoX.cr.cd.cell2Loc(nextCell))
        #     if dist < egoX.cr.cd.dx + 0.15 + 1 * egoX.vel_norm:
        #         u_b = (1/dist * 2*egoX.vel_norm)*2
        #     else: 
        #         u_b = 0
        # else: u_b = 0
        if self.nextCell != ():
            dist = egoX.location.distance(egoX.cr.cd.cell2Loc(self.nextCell))
            dt = egoX.cr.cd.traj.get(self.nextCell)[0] - worldX.tick.elapsed_seconds
            if dt > 0:
                velDes = dist/dt
            else:
                delay = dist/egoX.vel_norm
                egoX.cr.cd.traj[self.nextCell] = (egoX.cr.cd.traj.get(self.nextCell)[0]+delay,egoX.cr.cd.traj.get(self.nextCell)[1]+delay)
                velDes = egoX.vel_norm
        else:
            velDes = np.inf
        #* PID Control
        (u_v,self.vPIDStates) = velocityPID(egoX,self.vPIDStates,velDes)
        (u_theta,self.thetaPIDStates)  = anglePID(egoX,worldX,self.thetaPIDStates)
        u_b = 0
        try:
            # print(round(velDes,2),round(egoX.vel_norm,2),round(dist,2),round(dt,2),round(u_v,2))
            # print(round(velDes,2),round(egoX.vel_norm,2),self.currentCell)
            # print(round(dist,2),round(dt,2))
            # print(egoX.id,round(velDes,2),round(egoX.cr.cd.traj.get(self.nextCell)[0],2),round(egoX.cr.cd.traj.get(self.nextCell)[1],2))
            pass
        except:
            pass
        u_v = np.clip(u_v-u_b,-1,1)
        if u_v >= 0:
            ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0,manual_gear_shift=True,gear=1))
        elif u_v < 0:
            ego.apply_control(carla.VehicleControl(throttle=0, steer=u_theta,brake = - u_v,manual_gear_shift=True,gear=1))
        # TODO MPC CONTROL
        
        #* Emergency brake
        # cd_obj = cd.conflictDetection("coneDetect",[4,0.155*np.pi,5]).obj
        # for actor in worldX.actorDict.actor_list:
        #     if ego.id != actor.id and carla.Location.distance ( ego.get_location() , actor.get_location() ) < 12 : 
        #         if cd_obj.detect(ego,actor):
        #             ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0,brake = 1.0,manual_gear_shift=True,gear=1))

        state = egoX.state
        if egoX.state == "NAN":
            state = "IL"
        elif egoX.state == "IL": # and frontid something
            queue = [(egoX.id,egoX.location.distance(worldX.inter_location))]
            for msg in worldX.msg.inbox[egoX.id]:
                # If the sender is not in the intersection or leaving and on the same road_id:
                if msg.content.get("state") not in ["I","OL"] and msg.content.get("wp") != [] and msg.content.get("wp").road_id == egoX.waypoint.road_id:
                    queue.append((msg.idSend,worldX.actorDict.dict.get(msg.idSend).location.distance(worldX.inter_location)))
            queue.sort(key=lambda x: x[1])           
            if queue[0][0] == egoX.id:
                state = "FIL"
            else: 
                for pair in enumerate(queue):
                    if pair[1][0] == egoX.id:
                        egoX.infoSet("idFront",queue[pair[0] - 1][0])
                
                
        elif egoX.state == "FIL" and ego.get_location().distance(worldX.inter_location) <= egoX.cr.cd.size:
            state = "I"
        elif egoX.state == "I" and ego.get_location().distance(worldX.inter_location) >= egoX.cr.cd.size+1:
            state = "OL"
        egoX.discreteState(state)
        return state

class test:
    def __init__(self):
        self.vPIDStates = (0,0,0)
        self.thetaPIDStates = (0,0,0)
    def control(self,egoX,worldX):
        u_v = 1
        u_theta = 0
        egoX.ego.apply_control(carla.VehicleControl(throttle=u_v, steer=u_theta,brake = 0,manual_gear_shift=True,gear=1))




def velocityPID(egoX,states,velDes=np.inf):
    #* PID Gains < 
    kp = 0.575
    kd = 0.019
    ki = 0.44
    #* >
    v = np.linalg.norm([egoX.velocity.x,egoX.velocity.y])
    #* PID states <
    if velDes != np.inf:
        error = velDes - v
    else:
        error = egoX.velRef - v
    integral = states[1] + error * egoX.dt
    derivative = (error - states[0]) / egoX.dt
    #* >
    u_v = kp*(error) + ki*(integral) + kd*(derivative)
    states = (error,integral,derivative)
    return (u_v,states)

def anglePID(egoX,worldX,states):
    #* PID Gains
    kp = 2
    kd = 0
    ki = 0
    #* 
    
    # Make a list of the current + next 9 waypoints and sort it
    dist = []
    for i in range(10):
        try:
            dist.append([egoX.location.distance(egoX.route[egoX.routeIndex+i][0].transform.location),i])
        except:
            pass
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

def modelPredictiveControl():
    # Along path, so control is sdotdot, states are s and sdot
    # Finite horizon
    N = 5
    # State space:
    ss_A = matrix(np.array([[0,1],[0,0]]))
    ss_B = matrix(np.array([[0],[0]]))
    # Constraints
    A = matrix()
    b = matrix()
    Aeq = matrix()
    beq = matrix()
    # Cost function
    P = matrix([[],[]])
    q = matrix()
    # Convex Feasible Set 
    # 1/2 xT P x + qx
    # s.t.  G*x <= h
    #       A*x = b
    # for k in range(optim_iter):
    sol = solvers.qp(P,q,A,b,Aeq,beq)
    u = None    
    return u

