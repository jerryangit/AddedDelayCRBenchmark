#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import actorControl as ac
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
import time
import numpy as np
# Customizable stats
syncmode = 1
waves = 1 
random.seed(27)
# occupied by another object, the function will return None.
# 0 is random, 1 is 4 simultaneous
scenario = 1

## Create data folder

if not os.path.exists('./data'):
    os.makedirs('./data')

def main():
    # CONFIG
    maxVehicle = 30
    totalVehicle = 100
    
    # Initialize values
    i = 0
    iw = 0
    # spwnRand = random.randint(1,4)
    # destRand = random.randint(1,4)
    # Change randint if not 4 way cross road
    spwnRand = [random.randint(1, 4) for iter in range(totalVehicle)] 
    destRand = [random.randint(1, 4) for iter in range(totalVehicle)]
    spwnTime = []
    destTime = []
    actor_list = []


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

        # Map Locations, spwnLoc contains loc, 0= intersec, 1 = N, 2 = E, 3 = S, 4 = W.
        intersection = carla.Transform(carla.Location(x=-150, y=-35, z=0.3), carla.Rotation(yaw=180))
        northSpawn = carla.Transform(carla.Location(x=-151.3, y=-70, z=0.3), carla.Rotation(yaw=90))
        eastSpawn = carla.Transform(carla.Location(x=-115, y=-37, z=0.3), carla.Rotation(yaw=-180))
        southSpawn = carla.Transform(carla.Location(x=-149, y=0, z=0.3), carla.Rotation(yaw=-90))
        westSpawn = carla.Transform(carla.Location(x=-185, y=-33.7, z=0.3), carla.Rotation(yaw=0))
        spwnLoc = [intersection,northSpawn,eastSpawn,southSpawn,westSpawn]

        

        while i < totalVehicle and iw < waves:
            # TODO: Think about tickrate for control and simulation
            if syncmode == 1: 
                world.tick()
                tick = world.get_snapshot()
            else:
                tick = world.wait_for_tick()
            ts = tick.timestamp

            if False:
                bp = random.choice(blueprint_library.filter('vehicle'))
            else:
                bp = blueprint_library.find('vehicle.audi.a2')

            if scenario == 0:
                spwn = world.try_spawn_actor(bp, spwnLoc[spwnRand[i]])
                if spwn is not None:
                    actor_list.append(spwn)
                    # spwn.set_autopilot()
                    spwnTime.append(ts.elapsed_seconds-ts0s)
                    print('[%d] created %s at %d with id %d' % (i,spwn.type_id,spwnRand[i],spwn.id))
                    i += 1
                # Loop to destroy vehicles which are far away enough
                for actor in actor_list:
                    if actor.get_location().distance(carla.Location(x=-150, y=-35, z=0.3)) > 38 and actor.get_location().distance(carla.Location(x=0, y=0, z=0)) > 5:
                        print(actor.id,' left the area at ', round(actor.get_location().x,2),round(actor.get_location().y,2),round(actor.get_location().z,2))
                        destTime.append(ts.elapsed_seconds-ts0s)
                        actor_list.remove(actor)
                        actor.destroy() 

            if scenario == 1:
                k = 1
                while k <= 4:
                    spwn = world.try_spawn_actor(bp, spwnLoc[k])
                    if spwn is not None:
                        actor_list.append(spwn)
                        spwnTime.append(ts.elapsed_seconds-ts0s)
                        print('[%d] created %s at %d with id %d' % (i,spwn.type_id,spwnRand[i],spwn.id))
                        k += 1
                iw += 1


                for actor in actor_list:
                    if actor.get_location().distance(carla.Location(x=-150, y=-35, z=0.3)) > 38 and actor.get_location().distance(carla.Location(x=0, y=0, z=0)) > 5:
                        print(actor.id,' left the area at ', round(actor.get_location().x,2),round(actor.get_location().y,2),round(actor.get_location().z,2))
                        destTime.append(ts.elapsed_seconds-ts0s)
                        actor_list.remove(actor)
                        actor.destroy()


            # Loop to communicate information
            for actor in actor_list:
                pass
            # Loop to apply vehicle control
            map = world.get_map()
            j = 0
            for actor in actor_list:
                # Apply desired control function (control should be simple, precompute common info)
                # para = 0 # make class where para contains useful info
                w = map.get_waypoint(actor.get_location())
                info = ac.info(j,w,actor_list)
                ac.simpleControl(actor,info)
                j += 1



        # Destroy remaining actors after generation has finished
        while  len(actor_list) > 0:
            if syncmode == 1: 
                world.tick()
                tick = world.get_snapshot()
            else:
                tick = world.wait_for_tick()
            ts = tick.timestamp
            for actor in actor_list:
                if actor.get_location().distance(carla.Location(x=-150, y=-35, z=0.3)) > 38 and actor.get_location().distance(carla.Location(x=0, y=0, z=0)) > 5:
                    print(actor.id,' left the area at ', round(actor.get_location().x,2),round(actor.get_location().y,2),round(actor.get_location().z,2))
                    destTime.append(ts.elapsed_seconds-ts0s)
                    actor_list.remove(actor)
                    actor.destroy() 

            # Loop to communicate information
            for actor in actor_list:
                pass
            # Loop to apply vehicle control
            map = world.get_map()
            j = 0
            for actor in actor_list:
                # Apply desired control function (control should be simple, precompute common info)
                # para = 0 # make class where para contains useful info
                w = map.get_waypoint(actor.get_location())
                info = ac.info(j,w,actor_list)
                ac.simpleControl(actor,info)
                j += 1
        
        # print('spwnRand:',spwnRand)
        # print('destRand:',destRand)
        # print('spwnTime:',spwnTime)
        # print('destTime:',destTime)


    finally:
        # Save lists as csv
        data = []
        data = zip(spwnRand,destRand,spwnTime,destTime)
        filename = datetime.datetime.now().strftime('data-%Y-%m-%d-%H-%M.csv')
        with open('./data/'+filename, 'wb') as log:
            wr = csv.writer(log, quoting=csv.QUOTE_ALL)
            for row in data:
                wr.writerow(row)
        
        # Clean up actors
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')
        world.tick()



if __name__ == '__main__':

    main()
