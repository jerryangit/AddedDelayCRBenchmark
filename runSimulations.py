from Simulate import main as sim
import sys
import random

def main():
    startingIteration = 0
    for arg in sys.argv[1:]:
        startingIteration = int(arg)
    random.seed(0)
    totalVehicle = 4
    scenario = 0
    spwnInterval = 1.5
    logging = 0
    repetitions = 3
    scenarioList = []
    throughputList = [1.25,1,5/6,2/3,1/2,1/3] # vehicles per seconds
    # throughputList = [1.25,1,5/6,2/3] # vehicles per seconds
    totalVehicleList = [32,64,128]
    for i in range(repetitions):
        for totalVehicle in totalVehicleList:
            for throughput in throughputList:
                spwnInterval0 = 1/throughput
                spwnInterval1 = 4/throughput
                scenarioList.append((totalVehicle,0,spwnInterval0))
                scenarioList.append((totalVehicle,1,spwnInterval1))
    i = -1
    randomSeedList = [random.randint(0,1000000) for i in range(len(scenarioList))]
    for case in scenarioList:
        i += 1
        totalVehicle = case[0]
        scenario = case[1]
        spwnInterval = case[2]
        randomSeed = randomSeedList[i] # Generate randoms seed between 0 and 100000,logging
        if i < startingIteration:
            continue
        print("Running iteration: ", i, "Randomseed: ",randomSeed)
        # cr_method = "DCR"
        # ctrlPolicy = "DCRControl"
        # PriorityPolicy ="PriorityScore"
        # sim(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)

        # cr_method = "TEP_fix"
        # ctrlPolicy = "TEPControl"
        # PriorityPolicy ="FCFS"
        # sim(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)

        # cr_method = "MPIP"
        # ctrlPolicy = "MPIPControl"
        # PriorityPolicy ="FCFS"
        # sim(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)

        cr_method = "AMPIP"
        ctrlPolicy = "MPIPControl"
        PriorityPolicy ="FCFS"
        sim(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)
if __name__ == "__main__":
    main()