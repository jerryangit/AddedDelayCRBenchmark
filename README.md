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

## Descriptions

### Simulate.py
The main benchmarking file.
Arguments:
cr_method
ctrlPolicy
PriorityPolicy
totalVehicle
scenario
spwnInterval
randomSeed
preGenRoute
logging
### actorControl.py

### actorHelper.py

### benchmarkMetrics.py

### conflictDetection.py

### conflictResolution.py

### deadlockDetection.py

### mpc.py

### pathPlanner.py

### plotData.py

### priorityPolicy.py

### runSimulations.py

### topview.py
