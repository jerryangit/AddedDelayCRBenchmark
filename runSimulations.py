from main import main
import random
random.seed(0)
totalVehicle = 4
scenario = 0
spwnInterval = 1.5
logging = 0
repetitions = 3
scenarioList = []
throughputList = [2,1,2/3,1/3,1/6]
totalVehicleList = [16,32,64,128]
for i in range(repetitions):
    for totalVehicle in totalVehicleList:
        for throughput in throughputList:
            spwnInterval0 = 1/throughput
            spwnInterval1 = 4/throughput
            scenarioList.append((totalVehicle,0,spwnInterval0))
            scenarioList.append((totalVehicle,1,spwnInterval1))

for case in scenarioList:
    totalVehicle = case[0]
    scenario = case[1]
    spwnInterval = case[2]

    randomSeed = random.randint(0,1000000) # Generate randoms seed between 0 and 100000,logging
    cr_method = "DCR"
    ctrlPolicy = "DCRControl"
    PriorityPolicy ="PriorityScore"
    main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)

    cr_method = "TEP_fix"
    ctrlPolicy = "TEPControl"
    PriorityPolicy ="FCFS"
    main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)

    cr_method = "MPIP"
    ctrlPolicy = "MPIPControl"
    PriorityPolicy ="FCFS"
    main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)

    cr_method = "AMPIP"
    ctrlPolicy = "MPIPControl"
    PriorityPolicy ="FCFS"
    main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed,logging)