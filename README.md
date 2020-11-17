# # AddedDelayCRBenchamrk
This repository includes the benchmarks made for the thesis project of Jerry An (Decentralized Conflict Resolution for Autonomous Vehicles) using the CARLA PythonAPI.

## Requirements:
* CARLA Version 0.9.9 (including Town07) 
* Python 3.7.9

## Installation:
Clone the repository into your PythonAPI folder (i.e. 'PythonAPI/AddedDelayCRBenchmark')

## Usage:
1. Run CARLA with a fixed timestep equal to that used in 'Simulate.py'.
2. Run the benchmark using Simulate.py or run multiple at once using 'runSimulations.py'

See comments in source code for details on how to adjust conflict resolution methods.

## Executable Python files

### Simulate.py
The main benchmarking file.

### runSimulations.py
Used to run multiple simulations in a row.

### topview.py
Used to visualize CARLA in no_rendering_mode.

### benchmarkMetrics.py
Gets the added delay metrics

### plotData.py
Plot data for benchmarks

### playRecordings.py
Used to replay the CARLA recordings

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
Contains the mpc used by TDCR, uses cvxopt (suboptimal performance).

### mpc_oa.py
Contains the mpc used by OA-ADMM, uses osqp (faster but poor documentation and issues with scaling


## Other files

### pathPlanner.py
Replaced by carla GlobalRoutePlanner

### osqpTest.py
File used to test osqp

