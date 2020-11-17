# AddedDelayCRBenchamrk
This repository includes the benchmarks made for the thesis project of Jerry An (Decentralized Conflict Resolution for Autonomous Vehicles) using the CARLA PythonAPI.

**See OA-ADMM Branch for implementation with OA-ADMM !Master Branch is behind!**

## Requirements:
* CARLA Version 0.9.6 (including Town07)
* Python 3.7.9

## Installation:
Clone the repository into your PythonAPI folder (i.e. 'PythonAPI/AddedDelayCRBenchmark')

## Usage:
1. Run CARLA with a fixed timestep equal to that used in 'Simulate.py'.
2. Run the benchmark using Simulate.py or run multiple at once using 'runSimulations.py'

## Executable Python files

### Simulate.py
The main benchmarking file.
Arguments:
cr_method (Conflict Resolution Method)
ctrlPolicy (Control Policy)
PriorityPolicy (Priority Policy)
totalVehicle (Total amount of vehicles to be spawned)
scenario (Which benchmark scenario is ran)
spwnInterval (Interval between spawns)
randomSeed (Control Policy)
preGenRoute (Control Policy)
logging (Control Policy) 

### runSimulations.py
Used to run multiple simulations in a row.

### topview.py
Used to visualize CARLA in no_rendering_mode.

### benchmarkMetrics.py
Gets the added delay metrics

### plotData.py
Plot data for benchmarks

## Modules

### actorControl.py
Contains the various vehicle controllers.

### actorHelper.py
Contains helper classes for actor and world objects nececcary for conflict resolution calculations.

### conflictDetection.py
Contains the various conflict detection methods.

### conflictResolution.py
Contains the various conflict resolution methods.

### deadlockDetection.py
Contains the various deadlock detection methods.

### priorityPolicy.py
Contains the various priority policies.

### mpc.py
Contains the mpc used by TDCR.

## Other files
### pathPlanner.py
Replaced by carla GlobalRoutePlanner


