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
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
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
    syncmode = 1                # Whether ticks are synced
    random.seed(23)             # Random seed
    maxVehicle = 20             # Max simultaneous vehicle
    totalVehicle = 48           # Total vehicles for entire simulation
    scenario = 1                # 0 is random 1/tick, 1 is 4/tick all roads (Ensure totalVehicle is a multiple of 4 if scenario is 1)
    spwnInterval = 0.2            # Time between each spawn cycle
    cr_method = "AMPIP"          # Which conflict resolution method is used
    ctrlPolicy = "AMPIPControl"   # Which control policy is used
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
            settings = world.get_settings()
            if settings.synchronous_mode == False:
                settings.fixed_delta_seconds = 0.1
                settings.synchronous_mode = True
                settings.no_rendering_mode = False
                world.apply_settings(settings)
            world.tick()
            tick0 = world.get_snapshot()
        else:
            tick0 = world.wait_for_tick()
        #TODO are these used?
        ts0 = tick0.timestamp
        ts0s = tick0.timestamp.elapsed_seconds
        # ! Clear previous actors if present
        for actor in world.get_actors().filter("vehicle.*"):
            actor.destroy()

        # get map and establish grp
        map = world.get_map()
        hop_resolution = 0.25
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
        if scenario == 0: 
            kmax = 1
            spwnRand = np.array([random.choice(laneList) for iter in range(totalVehicle)])
            destRand = np.array([random.choice(np.delete(laneList,spwnRand[iter]-1)) for iter in range(totalVehicle)])
            velRand = np.array([8+random.uniform(-1.5,2) for iter in range(totalVehicle)])
        elif scenario == 1:
            kmax = 4
            spwnRand = np.array([[1,2,3,4] for iter in range(int(totalVehicle/4))]).flatten()
            destRand = np.array([random.choice(np.delete(laneList,spwnRand[iter]-1)) for iter in range(totalVehicle)])
            velRand = np.array([8+random.uniform(-1.5,2) for iter in range(totalVehicle)])
        elif scenario == 2: # Artificial AMPIP example
            totalVehicle = 2
            spwnInterval = 4.5
            kmax = 1
            spwnRand = [4,2]
            destRand = [1,1]
            velRand = [6,9]
        elif scenario == 3:
            # Artificial Deadlock for AMPIP, proof of not deadlock free in real environments
            totalVehicle = 4
            spwnInterval = 2.8
            kmax = 2
            spwnRand = [4,2,3,1]
            destRand = [2,4,1,3]
            velRand = [6.9,6.9,9.2,9.2]
        # idRand is only used for tie breaking, used to avoid odd behavior
        idRand = np.array([random.randint(100000,999999) for iter in range(totalVehicle)])
        spwnTime = [0]
        destTime = [0]

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
        worldX_obj = ah.worldX(world,intersection.location,8,tick0,hop_resolution)
        actorDict_obj = ah.actorDict()
        ctrl_obj = ac.actorControl(ctrlPolicy)
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
            worldX_obj.tock(tick)
            

            # TODO Spawn Vehicles code ( separate class or function)
            if ts.elapsed_seconds - ts0s - spwnTime[-1] > spwnInterval and i < totalVehicle and len(actorDict_obj.actor_list) <= maxVehicle:
                if False:
                    bp = random.choice(blueprint_library.filter('vehicle'))
                else:
                    bp = blueprint_library.find('vehicle.audi.a2')

                for _k in range(kmax):
                    spwn = world.try_spawn_actor(bp, spwnLoc[spwnRand[i]])
                    if spwn is not None:
                        # Add new spwn to actor_list
                        actorDict_obj.actor_list.append(spwn)
                        #// spwn.set_autopilot()
                        # Create inbox for new vehicle
                        worldX_obj.msg.inbox[spwn.id] = []
                        # Create actorX object for new vehicle
                        spwnX = ah.actorX(spwn,0,dt,exitLoc[destRand[i]],velRand[i],idRand[i])
                        # Trace route using A* and set to spwnX.route
                        spwnX.route = grp.trace_route(spwnLoc[spwnRand[i]].location,spwnX.dest.location)
                        # Create conflict resolution object and save it
                        spwnX.cr = cr.conflictResolution(cr_method,[0.5,"FCFS"]).obj
                        # Setup conflict resolution using egoX and worldX
                        spwnX.cr.setup(spwnX,worldX_obj)
                        # Add new objects to dictionary
                        actorDict_obj.addKey(spwn.id,spwnX)
                        # Add spawn time to list for analysis
                        spwnTime.append(ts.elapsed_seconds-ts0s)
                        # Print out to console
                        print('[%d,%d] created %s at %d with dest %d' % (i,spwn.id,spwn.type_id,spwnRand[i],destRand[i]))
                        i += 1

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
                actorX = actorDict_obj.dict.get(actor.id)
                worldX_obj.msg.receive(actor)
                actorX.updateParameters(actor,dt)
            worldX_obj.msg.clearCloud()

            for actor in actorDict_obj.actor_list:
                actorX = actorDict_obj.dict.get(actor.id)
                actorX.cr.resolve(actorX,worldX_obj)
                msg = actorX.cr.outbox(actorX)
                worldX_obj.msg.broadcast(actor,msg,50)
            # >>
            #* Loop to apply vehicle control (TODO separate class or function) 
            # <<
            j = 0
            for actor in actorDict_obj.actor_list:
                # Apply desired control function (control should be simple, precompute common info)
                # para = 0 # make class where para contains useful info
                actorX = actorDict_obj.dict.get(actor.id)
                actorX.discreteState(ctrl_obj.control(actorX,worldX_obj))
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
        print("setting synchronous_mode to False")
        settings.synchronous_mode = True
        world.apply_settings(settings)


def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print("Toc: start time not set")


if __name__ == '__main__':
    main()