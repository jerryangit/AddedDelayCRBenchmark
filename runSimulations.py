from Simulate import main as sim
import sys
import random

def main():
    startingIteration = 0
    if len(sys.argv) > 1:
        benchmark = sys.argv[1]
        if len(sys.argv) > 2:
            startingIteration = int(sys.argv[2])

    i = -1
    # benchmark = "default" 
    benchmark = "errMarginAMPIP"
    paraList = paraGen(benchmark)
    for para in paraList:
        i += 1
        if i < startingIteration:
            continue
        print("Running iteration: ", i, "Randomseed: ",para[6])
        sim(*para)

def paraGen(benchmark):
    paraList = []
    if benchmark == "default":
        random.seed(0)
        logging = 0
        repetitions = 3
        scenarioList = [0,1]
        throughputList = [1.25,1,5/6,2/3,1/2,1/3] # vehicles per seconds
        totalVehicleList = [32,64,128]
        preGenRoute = 1
        for rep in range(repetitions):
            for totalVehicle in totalVehicleList:
                for throughput in throughputList:
                    for scenario in scenarioList:
                        # Generate a random seed which is used for all para in this iteration
                        randomSeed = random.randint(1,999999)
                        # Set spwnInterval depending on scenario
                        if scenario == 0:
                            spwnInterval = 1/throughput
                        elif scenario == 1:
                            spwnInterval = 4/throughput
                        # Append para to paraList for the varying protocols
                        cr_method = "DCR"
                        ctrlPolicy = "DCRControl"
                        PriorityPolicy ="PriorityScore"
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging))

                        cr_method = "TEP"
                        ctrlPolicy = "TEPControl"
                        PriorityPolicy ="FCFS"
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging))

                        cr_method = "MPIP"
                        ctrlPolicy = "MPIPControl"
                        PriorityPolicy ="FCFS"
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging))

                        cr_method = "AMPIP"
                        ctrlPolicy = "MPIPControl"
                        PriorityPolicy ="FCFS"
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging))


    if benchmark == "errMarginAMPIP":
        random.seed(2)
        errMarginList = [0, 0.1, 0.2, 0.3, 0.4 ,0.5 ,0.6, 0.7, 0.8, 0.9, 1]
        logging = 0
        repetitions = 5
        scenarioList = [1]
        throughputList = [2/5] # vehicles per seconds
        totalVehicleList = [64]
        preGenRoute = 1        
        for rep in range(repetitions):
            for totalVehicle in totalVehicleList:
                for throughput in throughputList:
                    for scenario in scenarioList:
                        # Generate a random seed which is used for all para in this iteration
                        randomSeed = random.randint(1,999999)
                        # Set spwnInterval depending on scenario
                        if scenario == 0:
                            spwnInterval = 1/throughput
                        elif scenario == 1:
                            spwnInterval = 4/throughput
                        #! Note that this loop all uses the same randomSeed for better comparability
                        for errMargin in errMarginList:
                            # Append para to paraList for the varying protocols
                            # cr_method = "DCR"
                            # ctrlPolicy = "DCRControl"
                            # PriorityPolicy ="PriorityScore"
                            # paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin))

                            # cr_method = "TEP"
                            # ctrlPolicy = "TEPControl"
                            # PriorityPolicy ="FCFS"
                            # paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin))

                            # cr_method = "MPIP"
                            # ctrlPolicy = "MPIPControl"
                            # PriorityPolicy ="FCFS"
                            # paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin))

                            cr_method = "AMPIP"
                            ctrlPolicy = "MPIPControl"
                            PriorityPolicy ="FCFS"
                            paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin))
    return paraList
    
if __name__ == "__main__":
    main()