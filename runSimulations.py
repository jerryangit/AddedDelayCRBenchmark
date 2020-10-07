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
        random.seed(1)
        logging = 0
        repetitions = 3
        errMargin = 0.25
        scenarioList = [8]
        throughputList = [1] # vehicles per seconds
        totalVehicleList = [100]
        preGenRoute = 1
        parg = None

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
                        recordName = 'NoConflict_20_10_07.log'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, 9, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName,parg))

                        cr_method = "AMPIP"
                        ctrlPolicy = "MPIPControl"
                        PriorityPolicy ="FCFS"
                        recordName = 'AMPIP_20_10_07_low.log'                        
                        parg = 'gridLow'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName,parg))                        
                        recordName = 'AMPIP_20_10_07_med.log'
                        parg = None
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName,parg))                        
                        recordName = 'AMPIP_20_10_07_high.log'                        
                        parg = 'gridHigh'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName,parg))                        

                        cr_method = "DCR"
                        ctrlPolicy = "DCRControl"
                        PriorityPolicy ="PriorityScore"
                        recordName = 'DCR_20_10_07_low.log'
                        parg = 'gridLow'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName,parg))
                        recordName = 'DCR_20_10_07_med.log'
                        parg = None
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName,parg))                        
                        recordName = 'DCR_20_10_07_high.log'
                        parg = 'gridHigh'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName,parg))                        

                        cr_method = "OAADMM"
                        ctrlPolicy = "OAMPC"
                        PriorityPolicy ="PriorityScore"
                        recordName = 'OAADMM_20_10_07.log'
                        paraList.append((cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed, preGenRoute, logging, errMargin, recordName, parg))
    return paraList
if __name__ == "__main__":
    main()