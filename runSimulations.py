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
    benchmark = "default" 
    # benchmark = "errMarginAMPIP"
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
        logging = 1
        repetitions = 1
        errMargin = 1
        scenarioList = [8]
        throughputList = [1.25,1,5/6,2/3,1/2,1/3] # vehicles per seconds
        totalVehicleList = [0]
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
                        elif scenario == 8:
                            spwnInterval = 30
                        # Append para to paraList for the varying protocols
                        cr_method = "OAADMM"
                        ctrlPolicy = "OAMPC"
                        PriorityPolicy ="PriorityScore"
                        recordName = 'record_NoConflict_20_10_05_1.log'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, 9, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName))

                        cr_method = "AMPIP"
                        ctrlPolicy = "MPIPMPCControl"
                        PriorityPolicy ="FCFS"
                        recordName = 'record_AMPIP_20_10_05_1.log'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, errMargin, logging))

                        # cr_method = "DCR"
                        # ctrlPolicy = "DCRControl"
                        # PriorityPolicy ="PriorityScore"
                        # recordName = 'record_DCR_20_10_05_1.log'
                        # paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, errMargin, logging))

                        cr_method = "OAADMM"
                        ctrlPolicy = "OAMPC"
                        PriorityPolicy ="PriorityScore"
                        recordName = 'record_OAADMM_20_10_05_1.log'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName))
    return paraList
if __name__ == "__main__":
    main()