#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import actorControl as ac
import actorHelper as ah
# import pathPlanner as pp
import conflictResolution as cr
import glob
import os
import sys
import csv
import datetime
# Adds carla PythonAPI to path
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.5-%s.egg' % (
        sys.version_info.major,
        # sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Adds carla/agents module to path
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.global_route_planner import GlobalRoutePlanner

import random
import time
import numpy as np

## Create data folder
if not os.path.exists('./data'):
    os.makedirs('./data')

def main():
    ###############################################
    # Config
    ###############################################  
    syncmode = 1         # Whether ticks are synced
    random.seed(27)      # Random seed
    maxVehicle = 20      # Max simultaneous vehicle
    totalVehicle = 8    # Total vehicles for entire simulation
    scenario = 1         # 0 is random 1/tick, 1 is 4/tick all roads (Ensure totalVehicle is a multiple of 4 if scenario is 1)
    cr_method = "TEP"    # Which conflict resolution method is used
    ###############################################
    # Initialize values
    ###############################################  


    try:
        # Init carla at port 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve world
        world = client.get_world()
        if syncmode == 1: 
            world.tick()
            tick0 = world.get_snapshot()
        else:
            tick0 = world.wait_for_tick()
        ts0 = tick0.timestamp
        ts0s = tick0.timestamp.elapsed_seconds
        # ! Clear previous actors if present
        for actor in world.get_actors().filter("vehicle.*"):
            actor.destroy()

        # get map and establish grp
        map = world.get_map()
        hop_resolution = 0.5
        dao = GlobalRoutePlannerDAO(map, hop_resolution)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        # Retrive blueprints
        blueprint_library = world.get_blueprint_library()

        # Vehicle type definition
        if False:
            bp = random.choice(blueprint_library.filter('vehicle'))
        else:
            bp = blueprint_library.find('vehicle.audi.a2')

        # A blueprint contains the list of attributes that define a vehicle's
        # instance, we can read them and modify some of them. For instance,
        # let's randomize its color.
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        #* Generate spawn
        i = 0
        # spwnRand = random.randint(1,4)
        # destRand = random.randint(1,4)
        # Change laneList if not 4 way cross road
        # Method does not allow same lane for entry and exit
        laneList= np.array([1,2,3,4])
        spwnRand = np.array([random.choice(laneList) for iter in range(totalVehicle)])
        destRand = np.array([random.choice(np.delete(laneList,spwnRand[iter]-1)) for iter in range(totalVehicle)])
        spwnTime = []
        destTime = []

        # Integrate into map object?
        # Map Locations, spwnLoc contains loc, 0= intersec, 1 = N, 2 = E, 3 = S, 4 = W.
        intersection = carla.Transform(carla.Location(x=-150.0, y=-35.0, z=0.3), carla.Rotation(yaw=180))
        northSpawn = carla.Transform(carla.Location(x=-151.8, y=-70.0, z=0.3), carla.Rotation(yaw=90))
        eastSpawn = carla.Transform(carla.Location(x=-115.0, y=-37.0, z=0.3), carla.Rotation(yaw=-180))
        southSpawn = carla.Transform(carla.Location(x=-148.5, y=0.0, z=0.3), carla.Rotation(yaw=-90))
        westSpawn = carla.Transform(carla.Location(x=-185.0, y=-33.3, z=0.3), carla.Rotation(yaw=0))
        spwnLoc = [intersection,northSpawn,eastSpawn,southSpawn,westSpawn]
        
        northExit = carla.Transform(carla.Location(x=-148.5, y=-70.0, z=0.3), carla.Rotation(yaw=-90))
        eastExit = carla.Transform(carla.Location(x=-115.0, y=-33.3, z=0.3), carla.Rotation(yaw=0))
        southExit = carla.Transform(carla.Location(x=-151.8, y=0.0, z=0.3), carla.Rotation(yaw=90))
        westExit = carla.Transform(carla.Location(x=-185.0, y=-37.0, z=0.3), carla.Rotation(yaw=-180))
        exitLoc = [intersection,northExit,eastExit,southExit,westExit]

        # Create Objects to use in loop
        #<<
        worldX_obj = ah.worldX(world,intersection.location,8)
        actorDict_obj = ah.actorDict()
        #>>
        
        #*  << Main Loop >>
        
        notComplete = 1
        while notComplete: 
            if syncmode == 1: 
                world.tick()
                tick = world.get_snapshot()
            else:
                tick = world.wait_for_tick()
            ts = tick.timestamp
            dt = ts.delta_seconds
            
            # TODO Spawn Vehicles code ( separate class or function)
            if i < totalVehicle and len(actorDict_obj.actor_list) <= maxVehicle:
                if False:
                    bp = random.choice(blueprint_library.filter('vehicle'))
                else:
                    bp = blueprint_library.find('vehicle.audi.a2')
                if scenario == 0:
                    spwn = world.try_spawn_actor(bp, spwnLoc[spwnRand[i]])
                    if spwn is not None:
                        actorDict_obj.actor_list.append(spwn)
                        # spwn.set_autopilot()
                        worldX_obj.msg.inbox[spwn.id] = []
                        spwnX = ah.actorX(spwn,0,dt,exitLoc[destRand[i]])
                        route = grp.trace_route(spwnX.ego.get_location(),spwnX.dest.location,8.33)
                        spwnX.updatePath(route)
                        spwnX.conflictResolution(cr.conflictResolution("TEP",[0.5,"FCFS"]).obj) 
                        actorDict_obj.addKey(spwn.id,spwnX)
                        spwnTime.append(ts.elapsed_seconds-ts0s)
                        print('[%d] created %s at %d with id %d' % (i,spwn.type_id,spwnRand[i],spwn.id))
                        i += 1

                if scenario == 1:
                    k = 1
                    while k <= 4:
                        spwn = world.try_spawn_actor(bp, spwnLoc[k])
                        if spwn is not None:
                            actorDict_obj.actor_list.append(spwn)
                            worldX_obj.msg.inbox[spwn.id] = []
                            spwnX = ah.actorX(spwn,0,dt,exitLoc[random.choice(np.delete(laneList,k-1))],8.33)
                            route = grp.trace_route(spwnLoc[k].location,spwnX.dest.location)
                            spwnX.updatePath(route)
                            spwnX.conflictResolution(cr.conflictResolution("TEP",[0.5,"FCFS"]).obj) 
                            actorDict_obj.addKey(spwn.id,spwnX)
                            spwnTime.append(ts.elapsed_seconds-ts0s)                            
                            print('[%d] created %s at %d with id %d' % (i,spwn.type_id,spwnRand[i],spwn.id))
                            i += 1
                        k += 1

            #* Destroy Vehicles code 
            # TODO separate class or function
            for actor in actorDict_obj.actor_list:
                if actor.get_location().distance(carla.Location(x=-150, y=-35, z=0.3)) > 38 and actor.get_location().distance(carla.Location(x=0, y=0, z=0)) > 5:
                    print(actor.id,' left the area at ', round(actor.get_location().x,2),round(actor.get_location().y,2),round(actor.get_location().z,2))
                    destTime.append(ts.elapsed_seconds-ts0s)
                    actorDict_obj.actor_list.remove(actor)
                    del actorDict_obj.dict[actor.id]
                    worldX_obj.msg.clear(actor.id)
                    actor.destroy()

            # Feed world info to classes and functions streamlined
            worldX_obj.update(actorDict_obj)

            #* Loop to communicate information 
            # TODO separate class or function)
            # <<
            for actor in actorDict_obj.actor_list:
                worldX_obj.msg.receive(actor)

            worldX_obj.msg.clearCloud()

            for actor in actorDict_obj.actor_list:
                actorX = actorDict_obj.dict.get(actor.id)
                actorX.updateParameters(actor)
                actorX.cr.resolve(actorX,worldX_obj)
                msg = actorX.cr.outbox(actorX)
                worldX_obj.msg.broadcast(actor,msg)
            # >>


            #* Loop to apply vehicle control (TODO separate class or function) 
            # <<
            j = 0
            for actor in actorDict_obj.actor_list:
                # Apply desired control function (control should be simple, precompute common info)
                # para = 0 # make class where para contains useful info
                actorX = actorDict_obj.dict.get(actor.id)
                actorX.discreteState(ac.TEPControl(actorX,worldX_obj))
                actorX.tick(dt)
                # ac.simpleControl(actor,worldX_obj)
                j += 1
            # >>

            #* End the loop 
            # TODO Integrate in while loop if useless
            if i >= totalVehicle and len(actorDict_obj.actor_list) == 0:
                notComplete = 0


    finally:
        # Save lists as csv
        data = []
        data = zip(spwnRand,destRand,spwnTime,destTime)
        filename = datetime.datetime.now().strftime('data-%Y-%m-%d-%H-%M.csv')
        with open('./data/'+filename, 'w') as log:
            wr = csv.writer(log, quoting=csv.QUOTE_ALL)
            for row in data:
                wr.writerow(row)
        
        # Clean up actors
        print('destroying actors')
        for actor in actorDict_obj.actor_list:
            actor.destroy()
        print('done.')
        world.tick()



if __name__ == '__main__':

    main()