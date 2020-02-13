#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import numpy as np
import matplotlib.pyplot as plt
import os, sys, glob
import csv
from conflictDetection import generateSamples,detectCollision,affinePara
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.global_route_planner import GlobalRoutePlanner

if not os.path.exists('./metrics'):
    os.makedirs('./metrics')

def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        if world.get_map().name != 'Town07':
            client.load_world('Town07')
        else:
            client.reload_world()
        world = client.get_world()

        carlaMap = world.get_map()
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.audi.a2')
        actor = world.try_spawn_actor(bp, carla.Transform(carla.Location(x=-148.0, y=-25.0, z=0.31), carla.Rotation(yaw=-90)))
        
        settings = world.get_settings()
        if settings.synchronous_mode == False:
            settings.fixed_delta_seconds = 1/50
            settings.synchronous_mode = True
            settings.no_rendering_mode = True
            world.apply_settings(settings)
        world.tick()

        hop_resolution = 0.05
        dao = GlobalRoutePlannerDAO(carlaMap, hop_resolution)
        grp = GlobalRoutePlanner(dao)
        grp.setup()

        maneuverList = maneuverListGen()
        

        maneuverDistribution = maneuverDistributionGen(maneuverList)
        maneuverList,delayDict = noConflictListRemoval(maneuverList,maneuverDistribution)
        
        pathsDict = pathsDictGen(maneuverList,grp)

        # plotManeuver(maneuverList,maneuverList[1],pathsDict)

        sDict,sYieldDict,angDict = sDictGen(maneuverList,pathsDict,actor)

        delayDict,delayAvg = marginalDelay(maneuverList,sYieldDict,angDict,delayDict)
        maxThroughput = 1/delayAvg

    finally:
        pass

def marginalDelay(maneuverList,sYieldDict,angDict, delayDict={}, vRef = 7,throughput = 1):
    tmax = 0
    for maneuver in maneuverList:
        if maneuver in sYieldDict:
            for smp in enumerate(sYieldDict.get(maneuver)):
                tmax = tmax + smp[1] / ( vRef * np.cos( np.abs( 1/2 * np.pi - angDict.get( maneuver )[smp[0]] ) ) )
            delayDict[maneuver] = tmax/len(sYieldDict.get(maneuver))
    delayAvg = np.average(delayDict.values())
    return delayDict,delayAvg

def maneuverListGen():
    maneuverList = []       # Comments are for a 4 way intersection
    actionList_i = [0,1,2]  # 0 = right, 1 = forward, 2 = left, counterclockwise dir ascending
    roadList_j = [0,1,2,3]  # 0 = right road, 1 = forward road, 2 = left road, 3 = follower, roads are relative to i vehicle
    actionList_j = [0,1,2]  # 0 = right, 1 = forward, 2 = left, counterclockwise dir
    for action_i in actionList_i:
        for road_j in roadList_j:
            for action_j in actionList_j:
                # Maneuver = (a,b,c), a = yielding agent action, b is other agent road, c is other agent action
                maneuverList.append((action_i,road_j,action_j))   
    return maneuverList
    
def noConflictListRemoval(maneuverList,maneuverDistribution,delayDict={}):
    # Hardcoded for 4 way intersection
    noConflictList = [(1,1,0), (1,1,1), (1,2,0), (0,0,0), (0,0,1), (0,0,2), (0,1,0), (0,1,1), (0,2,0), (0,2,2), (2,0,0), (2,2,0),(0,3,0),(1,3,1),(2,3,2)]
    for maneuver in noConflictList :
        if maneuver in maneuverList:
            maneuverList.remove(maneuver)
            delayDict[maneuver] = (0)
    return maneuverList,delayDict

def maneuverDistributionGen(maneuverList):
    maneuverDistribution = {}
    for maneuver in maneuverList:
        maneuverDistribution[maneuver] = 1/len(maneuverList)
    return maneuverDistribution

def pathsDictGen(maneuverList,grp):
    intersection = carla.Transform(carla.Location(x=-150.0, y=-35.0, z=0.3), carla.Rotation(yaw=180))

    northSpawn = carla.Transform(carla.Location(x=-151.73236083984375, y=-45.01227569580078, z=0.21408706903457642), carla.Rotation(yaw=90))
    eastSpawn = carla.Transform(carla.Location(x=-140.02786254882812, y=-36.476444244384766, z=0.09466663748025894), carla.Rotation(yaw=-180))
    southSpawn = carla.Transform(carla.Location(x=-148.87168884277344, y=-24.99028968811035, z=0.2630075216293335), carla.Rotation(yaw=-90))
    westSpawn = carla.Transform(carla.Location(x=-160.00350952148438, y=-33.531150817871094, z=0.22501225769519806), carla.Rotation(yaw=0))
    spwnLoc = [eastSpawn,northSpawn,westSpawn,southSpawn]
    
    northExit = carla.Transform(carla.Location(x=-148.63150024414062, y=-45.064308166503906, z=0.21326427161693573), carla.Rotation(yaw=-90))
    eastExit = carla.Transform(carla.Location(x=-140.41676330566406, y=-33.379364013671875, z=0.09794315695762634), carla.Rotation(yaw=0))
    southExit = carla.Transform(carla.Location(x=-151.9658203125, y=-25.500608444213867, z=0.26464682817459106), carla.Rotation(yaw=90))
    westExit = carla.Transform(carla.Location(x=-159.53302001953125, y=-36.62759780883789, z=0.22634325921535492), carla.Rotation(yaw=-180))
    exitLoc = [eastExit,northExit,westExit,southExit]

    routeDictionary = {}
    for _spwnLoc in enumerate(spwnLoc):
        for _exitLoc in enumerate(exitLoc):   
            if _spwnLoc[0] != _exitLoc[0]:
                routeDictionary[(_spwnLoc[0],_exitLoc[0])] = grp.trace_route(_spwnLoc[1].location,_exitLoc[1].location)
    
    pathsDict = {}
    for maneuver in maneuverList:
        pathsDict[maneuver] = ((routeDictionary.get((3,maneuver[0]))),(routeDictionary.get((maneuver[1],action2loc(maneuver[1]+1+maneuver[2])))))
    return pathsDict

def plotManeuver(maneuverList,maneuver,pathsDict):
    plt.figure(0)
    plt.title(str(maneuver))
    xA = []
    yA = []
    for wp in pathsDict.get(maneuver)[0]:
        xA.append(wp[0].transform.location.x)
        yA.append(wp[0].transform.location.y)
    xB = []
    yB = []
    for wp in pathsDict.get(maneuver)[1]:
        xB.append(wp[0].transform.location.x)
        yB.append(wp[0].transform.location.y)

    plt.plot(xA,yA,'-')
    plt.plot(xB,yB,'r:')
    plt.xlim(-165, -135)
    plt.ylim(-50,-20)
    plt.gca().invert_yaxis()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def action2loc(actionNr):
    while actionNr > 3:
        actionNr += -4
    while actionNr < 0:
        actionNr += 4
    return actionNr


def sDictGen(maneuverList,pathsDict,actor,sYieldDict={},angDict={}):
    sDict = {}
    for maneuver in maneuverList:
        for wp_i in enumerate(pathsDict.get(maneuver)[0]):
            if wp_i[0] < len(pathsDict.get(maneuver)[0])-2:
                wp_i1 = pathsDict.get(maneuver)[0][wp_i[0]+1][0]
                smpList_i = generateSamples(actor,0,wp_i[1][0].transform)
                vec_i = np.array((wp_i1.transform.location.x-wp_i[1][0].transform.location.x,wp_i1.transform.location.y-wp_i[1][0].transform.location.y))
                isColl = 0
            for wp_j in enumerate(pathsDict.get(maneuver)[1]):
                if wp_j[0]<len(pathsDict.get(maneuver)[1])-2 and wp_i[1][0].transform.location.distance(wp_j[1][0].transform.location) < 2.5:
                    wp_j1 = pathsDict.get(maneuver)[1][wp_j[0]+1][0]
                    smpList_j = generateSamples(actor,0,wp_j[1][0].transform)
                    vec_j = np.array((wp_j1.transform.location.x-wp_j[1][0].transform.location.x,wp_j1.transform.location.y-wp_j[1][0].transform.location.y))
                    isColl = detectCollision(smpList_i,smpList_j)
                    if isColl == 1:
                        rej = vec_j - ( np.dot(vec_j,vec_i) )/( np.dot(vec_i,vec_i) )*vec_i
                        ang = np.arccos( np.dot(vec_j,vec_i)/(np.linalg.norm(vec_j)*np.linalg.norm(vec_i))  )
                        sDict[maneuver,wp_i[0],wp_j[0]] = (rej,ang)
                        if maneuver in sYieldDict.keys():
                            sYieldDict.get(maneuver).append(np.linalg.norm(rej))
                        else:   
                            sYieldDict[maneuver] = [np.linalg.norm(rej)]
                        if maneuver in angDict.keys():
                            angDict.get(maneuver).append(ang)
                        else:   
                            angDict[maneuver] = [ang]
                        break
    return sDict,sYieldDict,angDict


    
if __name__ == '__main__':
    main()