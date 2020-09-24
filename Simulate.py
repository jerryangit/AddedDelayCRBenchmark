#!/usr/bin/env python

# Copyright (c) 2019 Jerry An
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#! Multiprocessing
import actorControl as ac
import actorHelper as ah
# import pathPlanner as pp
import copyreg, pickle,copy
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
if not os.path.exists('./recordings'):
    os.makedirs('./recordings')

# def main(cr_method = "AMPIP", ctrlPolicy = "MPIPControl", PriorityPolicy = "FCFS",totalVehicle = 32, scenario = 0, spwnInterval = 0.8, randomSeed = 823182,logging = 1):
# def main(cr_method = "AMPIP", ctrlPolicy = "MPIPControl", PriorityPolicy = "FCFS",totalVehicle = 128, scenario = 0, spwnInterval = 0.8, randomSeed = 469730,logging = 1):
# def main(cr_method = "AMPIP", ctrlPolicy = "MPIPControl", PriorityPolicy = "FCFS",totalVehicle = 128, scenario = 0, spwnInterval = 1.2, randomSeed = 960489,logging = 1):

def main(cr_method = "OAADMM", ctrlPolicy = "OAMPC", PriorityPolicy = "PriorityScore",totalVehicle = 12, scenario = 7, spwnInterval = 1.75, randomSeed = 960489, preGenRoute = 1, logging = 1, errMargin = 0.5):
    ###############################################
    # Config
    ###############################################  
    syncmode = 1                # Whether ticks are synced
    freqSimulation = 100        # [HZ] The frequency at which the simulation is ran 
    freqOnBoard = 10            # [HZ] The frequency at which vehicle on board controller is simulated
    freqControl = 25           # [Hz] The frequency at which the low level control is performed
    random.seed(randomSeed)     # Random seed
    maxVehicle = 24             # Max simultaneous vehicle
    # preGenRoute 
    # logging = 1                 # Whether to log vehicle spawns and dest
    # totalVehicle = 64           # Total vehicles for entire simulation
    # scenario = 6                # 0 is random 1/tick, 1 is 4/tick all roads (Ensure totalVehicle is a multiple of 4 if scenario is 1)
    # spwnInterval = 4            # Time between each spawn cycle
    # cr_method = "DCR"                   # Which conflict resolution method is used
    # ctrlPolicy = "DCRControl"           # Which control policy is used
    # PriorityPolicy = "PriorityScore"    # Which priorityPolicy is used
    # cr_method = "MPIP"         # Which conflict resolution method is used
    # ctrlPolicy = "MPIPControl" # Which control policy is used
    # PriorityPolicy = "FCFS"    # Which priorityPolicy is used
    ###############################################
    # Plotting Config
    ###############################################  
    plot = 0                    # Whether to plot figures afterwards or not
    plotVel = 0                 # Whether to plot velocity or not
    plotTheta = 0               # Whether to plot theta or not
    plotLoc = 0                 # Whether to plot location or not
    record = 1

    ###############################################
    # Other variables
    ###############################################  
    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        precipitation_deposits=0.0, 
        wind_intensity=0.0, 
        sun_azimuth_angle=70.0, 
        sun_altitude_angle=70.0)                  # Doesn't affect simulation, but does affect visuals

    try:
        # Init carla at port 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve world
        world = client.get_world()
        if world.get_map().name != 'Town07':
            client.load_world('Town07')
        else:
            client.reload_world()
        world = client.get_world()
        world.get_spectator().set_transform(carla.Transform(carla.Location(x=-135.0, y=-22.50, z=20.), carla.Rotation(yaw=180))) 
        world.set_weather(weather)
        if syncmode == 1: 
            settings = world.get_settings()
            if settings.synchronous_mode == False:
                settings.fixed_delta_seconds = 1/freqSimulation
                settings.synchronous_mode = True
                settings.no_rendering_mode = True
                world.apply_settings(settings)
            world.tick()
            tick0 = world.get_snapshot()
        else:
            tick0 = world.wait_for_tick()

        if record == 1:
            # client.start_recorder("/home/jerry/carla/PythonAPI/conflictResolutionCarla/recordings/recording01.log")
            client.start_recorder('record_20_9_18_1.log')
        #TODO are these used?
        ts0 = tick0.timestamp
        ts0s = tick0.timestamp.elapsed_seconds
        lastTick_s = ts0s
        # ! Clear previous actors if present
        for actor in world.get_actors().filter("vehicle.*"):
            actor.destroy()

        # get map and establish grp
        carlaMap = world.get_map()
        hop_resolution = 0.1
        dao = GlobalRoutePlannerDAO(carlaMap, hop_resolution)
        grp = GlobalRoutePlanner(dao)
        grp.setup()

        # Retrive blueprints
        blueprint_library = world.get_blueprint_library()

        # Vehicle type definition
        if False:
            bp = random.choice(blueprint_library.filter('vehicle'))
        else:
            bp = blueprint_library.find('vehicle.audi.a2')

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
            velRand = np.array([5+random.uniform(-1,1) for iter in range(totalVehicle)])
        elif scenario == 1:
            kmax = 4
            spwnRand = np.array([[1,2,3,4] for iter in range(int(totalVehicle/4))]).flatten()
            destRand = np.array([random.choice(np.delete(laneList,spwnRand[iter]-1)) for iter in range(totalVehicle)])
            velRand = np.array([5+random.uniform(-1,1) for iter in range(totalVehicle)])
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
        elif scenario == 4:
            # fast spawning to test spawn delay
            kmax = 1
            totalVehicle = 8
            spwnInterval = 1.5
            # spwnRand = [1,2,3,4,1,2,3,4]
            # destRand = [4,1,2,3,4,1,2,3] 
            spwnRand = np.ones(totalVehicle, dtype = int)*2
            destRand = np.ones(totalVehicle, dtype = int)*4
            velRand = np.array([8+random.uniform(-1,1) for iter in range(totalVehicle)])
        elif scenario == 5:
            # testing for DCR
            kmax = 4
            totalVehicle = 4
            spwnInterval = 0
            spwnRand = [1,2,3,4]
            destRand = [3,4,1,2]
            velRand = [8,8,8,8]
            # velRand = np.array([8+random.uniform(-1,1) for iter in range(totalVehicle)])
        elif scenario == 6:
            # testing for theta PID
            kmax = 1
            totalVehicle = 1
            spwnInterval = 0
            spwnRand = [3]
            destRand = [1]
            velRand = [7]
        elif scenario == 7:
            # testing for OA-ADMM MPC
            kmax = 1
            totalVehicle = 8
            spwnInterval = 1.3
            spwnRand = [1,4,1,4,2,1,3,2,4,3,1,3,2,4,1,2,3,4,1,2,3,4,1,2,3,4]
            destRand = [3,2,3,3,4,4,1,1,2,2,2,4,3,1,2,3,4,1,3,4,1,2,4,1,2,3]
            velRand = np.array([5+0.5*random.uniform(-1,1) for iter in range(totalVehicle)])

        elif scenario == 8:
            # testing for OA-ADMM MPC Simultaneous
            kmax = 2
            totalVehicle = 2
            spwnInterval = 4.5
            spwnRand = [1,3,2,4,1,2,3,4]
            destRand = [3,1,4,2,3,4,1,2]
            velRand = [5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5]

        # idRand is only used for tie breaking, used to avoid odd behavior
        idRand = np.array([random.randint(100000,999999) for iter in range(totalVehicle)])
        spwnTime = [-spwnInterval]
        destTime = [-1]

        # Integrate into map object?
        # Map Locations, spwnLoc contains loc, 0= intersec, 1 = N, 2 = E, 3 = S, 4 = W.
        intersection = carla.Transform(carla.Location(x=-150.0, y=-35.0, z=0.3), carla.Rotation(yaw=180))
        # northSpawn = carla.Transform(carla.Location(x=-151.9, y=-70.0, z=0.272), carla.Rotation(yaw=90))
        # eastSpawn = carla.Transform(carla.Location(x=-115.0, y=-36.6, z=0.265), carla.Rotation(yaw=-180))
        # southSpawn = carla.Transform(carla.Location(x=-148.49, y=0.0, z=0.31), carla.Rotation(yaw=-90))
        # westSpawn = carla.Transform(carla.Location(x=-185.0, y=-33.3, z=0.01), carla.Rotation(yaw=0))
        northSpawn = carla.Transform(carla.Location(x=-151.49, y=-70.0, z=0.271), carla.Rotation(yaw=90))
        eastSpawn = carla.Transform(carla.Location(x=-115.05, y=-36.4, z=0.264), carla.Rotation(yaw=-180))
        southSpawn = carla.Transform(carla.Location(x=-149.05, y=00.0, z=0.305), carla.Rotation(yaw=-90))
        westSpawn = carla.Transform(carla.Location(x=-184.0, y=-33.7, z=0.0112), carla.Rotation(yaw=0))


        spwnLoc = [intersection,northSpawn,eastSpawn,southSpawn,westSpawn]
        
        northExit = carla.Transform(carla.Location(x=-148.2, y=-80.0, z=0.3), carla.Rotation(yaw=-90))
        eastExit = carla.Transform(carla.Location(x=-105.0, y=-33.3, z=0.3), carla.Rotation(yaw=0))
        southExit = carla.Transform(carla.Location(x=-152, y=10.0, z=0.3), carla.Rotation(yaw=90))
        westExit = carla.Transform(carla.Location(x=-195.0, y=-37.0, z=0.3), carla.Rotation(yaw=-180))
        exitLoc = [intersection,northExit,eastExit,southExit,westExit]

        # Pre generate the routes for all spawn and exit combinations
        if preGenRoute == 1:
            routeDictionary = {}
            for _spwnLoc in enumerate(spwnLoc):
                for _exitLoc in enumerate(exitLoc):   
                    if _spwnLoc[0] > 0 and _exitLoc[0] > 0 and _spwnLoc[0] != _exitLoc[0]:
                        routeDictionary[(_spwnLoc[0],_exitLoc[0])] = grp.trace_route(_spwnLoc[1].location,_exitLoc[1].location)

        # Create Objects to use in loop
        #<<
        worldX_obj = ah.worldX(world,intersection.location,8,tick0,hop_resolution)
        actorDict_obj = ah.actorDict()
        # ctrl_obj = ac.actorControl(ctrlPolicy)
        #>>
        
        # Random variables
        # TODO Clean up
        lastCycle_ob = 1
        lastCycle_ct = 1
        justSpwn = []
        if plot == 1:
            velDict = {}
            locDict = {}
            thetaDict = {}
            aclDict = {}
            ctrDict = {}

        print("Initialized with Method:",cr_method,", Control Policy: ", ctrlPolicy, ", Total Vehicles: ", totalVehicle, "Scenario: ",scenario, "Spawn Interval: ",spwnInterval, "Random Seed: ",randomSeed)

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
                    if i <= totalVehicle:
                        spwn = world.try_spawn_actor(bp, spwnLoc[spwnRand[i]])
                        if spwn is not None:
                            # Add new spwn to actor_list
                            actorDict_obj.actor_list.append(spwn)

                            # Create inbox for new vehicle
                            worldX_obj.msg.inbox[spwn.id] = []

                            # Create actorX object for new vehicle
                            spwnX = ah.actorX(spwn,0,dt,ts.elapsed_seconds-ts0s,spwnLoc[spwnRand[i]],spwnRand[i],exitLoc[destRand[i]],destRand[i],velRand[i],idRand[i],i)

                            # Trace route using A* and set to spwnX.route
                            if preGenRoute == 1:
                                spwnX.route = routeDictionary.get((spwnRand[i],destRand[i]))
                            else:
                                spwnX.route = grp.trace_route(spwnLoc[spwnRand[i]].location,spwnX.dest.location)

                            # Create conflict resolution object and save it
                            spwnX.cr = cr.conflictResolution(cr_method,[errMargin,PriorityPolicy]).obj

                            # Setup conflict resolution using egoX and worldX
                            spwnX.cr.setup(spwnX,worldX_obj)

                            # Create control object and save it
                            spwnX.co = ac.actorControl(ctrlPolicy)

                            # Add new objects to dictionary
                            actorDict_obj.addKey(spwn.id,spwnX)

                            # Add spawn time to list for analysis
                            spwnTime.append(ts.elapsed_seconds-ts0s)

                            # Set vehicle velocity to its reference
                            vel3D = proj3D(velRand[0],np.radians(spwnLoc[spwnRand[i]].rotation.yaw))
                            spwn.set_velocity(vel3D)
                            justSpwn.append((spwnX,vel3D))

                            # Set gear to 1 to avoid spawn delay bug
                            spwn.apply_control(carla.VehicleControl(manual_gear_shift=True,gear=1))

                            # Set gear back to automatic 
                            # spwn.apply_control(carla.VehicleControl(manual_gear_shift=False))
                            # Print out to console
                            if logging == 1:
                                print('[%d,%d] created %s at %d with dest %d, elapsed time: %.2f s' % (i,spwn.id,spwn.type_id,spwnRand[i],destRand[i],spwnTime[i+1]))
                            i += 1

                            if plot == 1:
                                if plotVel == 1:
                                    #* To Graph out velocities over time
                                    velDict[spwnX.id] = []
                                    ctrDict[spwnX.id] = []
                                    aclDict[spwnX.id] = []
                                if plotLoc == 1:
                                    #* To Graph out location over time
                                    locDict[spwnX.id] = []
                                if plotTheta == 1:
                                    thetaDict[spwnX.id] = []

            #* Destroy Vehicles code 
            # TODO separate class or function
            for actor in actorDict_obj.actor_list:
                if actor.get_location().distance(carla.Location(x=-150, y=-35, z=0.3)) > 38 and actor.get_location().distance(carla.Location(x=0, y=0, z=0)) > 5:
                    if logging == 1:
                        print('[',actorDict_obj.dict.get(actor.id)._spwnNr,',',actor.id,'] left the area at (', round(actor.get_location().x,2),', ',round(actor.get_location().y,2), '), elapsed time: ', round(ts.elapsed_seconds-ts0s), "s",sep='')
                    destTime.append(ts.elapsed_seconds-ts0s)
                    actorDict_obj.actor_list.remove(actor)
                    del actorDict_obj.dict[actor.id]
                    worldX_obj.msg.clear(actor.id)
                    actor.destroy()

            for actorX in actorDict_obj.dict.values():
                actorX.updateStats()

            #* Code to enforce a different freq for on board calculations and simulation


            # If last cycle of onboard has been geq to the expected ratio perform onboard
            if lastCycle_ob  >= freqSimulation/freqOnBoard: 
                lastCycle_ob = 1
                currTick_s = ts.elapsed_seconds
                dt_ob = currTick_s - lastTick_s
                lastTick_s = currTick_s
                # Feed world info to classes and functions streamlined      
                worldX_obj.update(actorDict_obj)

                #* Loop to communicate information 
                # <<
                for actorX in actorDict_obj.dict.values():
                    worldX_obj.msg.receive(actorX)
                    actorX.updateParameters(dt_ob)

                worldX_obj.msg.clearCloud()
                #* Loop to resolve conflicts 
                # <<
                # If OA-ADMM use multiple communication rounds
                if cr_method == "OAADMM":
                    # X-update, Send X-traj
                    for actorX in actorDict_obj.dict.values():
                        actorX.cr.xUpdate(actorX,worldX_obj)
                        msg = actorX.cr.outbox(actorX,"xUpdate")
                        worldX_obj.msg.broadcast(actorX.id,actorX.location,msg,250)
                    # Receive X-Traj
                    for actorX in actorDict_obj.dict.values():
                        worldX_obj.msg.receive(actorX)
                    worldX_obj.msg.clearCloud()
                    # Z-update, Lambda-update, Rho-update, and Send Z,Lambda,Rho
                    for actorX in actorDict_obj.dict.values():                    
                        actorX.cr.zUpdate(actorX,worldX_obj)
                        msg = actorX.cr.outbox(actorX,"zUpdate")
                        worldX_obj.msg.broadcast(actorX.id,actorX.location,msg,250)
                else:
                    for actorX in actorDict_obj.dict.values():
                        actorX.cr.resolve(actorX,worldX_obj)            
                        msg = actorX.cr.outbox(actorX)
                        worldX_obj.msg.broadcast(actorX.id,actorX.location,msg,250)
                # >>
            else: 
                lastCycle_ob += 1

            # If last cycle of onboard has been geq to the expected ratio perform onboard
            if lastCycle_ct >= freqSimulation/freqControl: 
                lastCycle_ct = 1
                #* Loop to apply vehicle control
                # <<
                for actorX in actorDict_obj.dict.values():
                    actorX.co.control(actorX,worldX_obj)
                # >>
                #* End the loop 
            else:
                lastCycle_ct += 1


            #* Set vehicle velocity to reference velocity for its first second
            for actorX,vel3D in justSpwn:
                if ts.elapsed_seconds-ts0s - actorX.spwnTime > 1.0:
                    justSpwn.remove((actorX,vel3D))
                    continue
                actorX.ego.set_velocity(vel3D)
                actorX.ego.apply_control(carla.VehicleControl(throttle=1, steer=0,brake = 0,manual_gear_shift=True,gear=1))

            # TODO Integrate in while loop if useless
            if i >= totalVehicle and len(actorDict_obj.actor_list) == 0:
                notComplete = 0
            # if logging == 1:
            #     worldX_obj0 = copy.copy(worldX_obj)
    finally:
        print("Ended with Method:",cr_method,", Control Policy: ", ctrlPolicy, ", Total Vehicles: ", totalVehicle, "Scenario: ",scenario, "Spawn Interval: ",spwnInterval, "Random Seed: ",randomSeed)

        # Save lists as csv
        data = []
        spwnTime.remove(-spwnInterval)
        destTime.remove(-1)
        data = zip(spwnRand,destRand,spwnTime,destTime)
        filename = str(cr_method) + "_" + str(ctrlPolicy) + "_" + str(totalVehicle).zfill(3) + "_" + str(scenario) + "_" + str(spwnInterval)+ "_" + str(randomSeed).zfill(6) + "_" + f'{errMargin:.1f}'.zfill(2) + "_" + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M.csv')
        dirname = './data/'
        with open(dirname+filename, 'w') as log:
            wr = csv.writer(log, quoting=csv.QUOTE_ALL)
            for row in data:
                wr.writerow(row)
        print("Log saved in: ",dirname+filename,sep="")

        timespent = []
        for i_s in enumerate(spwnTime):
            timespent.append(destTime[i_s[0]]-i_s[1])
        averageTime = np.average(timespent)
        print("Average time per vehicle: ", averageTime, "Total time:", destTime[-1])

        # Clean up actors
        print('destroying actors')
        for actor in actorDict_obj.actor_list:
            actor.destroy()
        print('done.')
        world.tick()
        client.stop_recorder()
        world.tick()

        # print("setting synchronous_mode to False")
        # settings.synchronous_mode = False
        # world.apply_settings(settings)
        # tick = world.wait_for_tick()
        
        #! Old stuff, used for debugging controller
        # if plot == 1:
        #     import matplotlib.pyplot as plt
        #     if plotTheta == 1:
        #         for value in thetaDict.values():
        #             t = [value[k][0] for k in range(len(value))]
        #             theta = [value[k][1] for k in range(len(value))]
        #             u = [value[k][2] for k in range(len(value))]
        #             fig, (ax1, ax2) = plt.subplots(2,1,num=10)
        #             fig.suptitle('Theta and steer input over time')
        #             ax1.plot(t,theta)
        #             ax2.plot(t,u)
        #     plt.show()

        #     if plotVel == 1:
        #         plt.figure()
        #         i = 0
        #         for value in velDict.values():
        #             t = [value[k][0] for k in range(len(value))]
        #             v = [value[k][1] for k in range(len(value))]
        #             plt.plot(t,v)
        #             plt.axhline(velRand[i],0,1)
        #             i += 1

        #         for value in ctrDict.values():
        #             t = [value[k][0] for k in range(len(value))]
        #             throttle = [value[k][1] for k in range(len(value))]
        #             steer = [value[k][2] for k in range(len(value))]
        #             brake = [value[k][3] for k in range(len(value))]
        #             gear = [value[k][4] for k in range(len(value))]
        #             manual = [value[k][5] for k in range(len(value))]
        #             figCtr, axCtr = plt.subplots(4,1)
        #             axCtr[0].plot(t,throttle)
        #             axCtr[0].set_ylim([0,1.1])
        #             axCtr[1].plot(t,steer)
        #             axCtr[1].set_ylim([-1,1])
        #             axCtr[2].plot(t,brake)
        #             axCtr[2].set_ylim([0,1.1])
        #             axCtr[3].plot(t,gear)
        #             axCtr[3].set_ylim([0,6])
        #             axCtr[3].plot(t,manual)
        #             axCtr[0].set_title('Control Inputs')

        #         for value in aclDict.values():
        #             plt.figure()
        #             t = [value[k][0] for k in range(len(value))]
        #             a = [np.linalg.norm([value[k][1],value[k][2]]) for k in range(len(value))]
        #             plt.title('Acceleration / time (m/s^2)')
        #             p_a = plt.plot(t,a)
        #             p_v = plt.plot(t,v)
        #             p_th = plt.plot(t,throttle)
        #             p_br = plt.plot(t,brake)
        #             # ax_br.set_ylim([0,15])
        #             ax = plt.gca()
        #             ax.set_ylim([0,16])
        #             plt.legend(["Acceleration", "Velocity","Throttle","Brake"])

                    
        #     if plotLoc == 1:
        #         for value in locDict.values():
        #             t = [value[k][0] for k in range(len(value))]
        #             x = [value[k][1] for k in range(len(value))]
        #             y = [value[k][2] for k in range(len(value))]
        #             fig, (ax1, ax2) = plt.subplots(2)
        #             fig.suptitle('X and Y over time (m)')
        #             ax1.plot(t,x)
        #             ax2.plot(t,y)
        #     plt.show()
        #     pass



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

def proj3D(r,theta):
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    z = 0
    return carla.Vector3D(x,y,z)

if __name__ == '__main__':
    main()