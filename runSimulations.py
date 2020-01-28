from main import main


totalVehicle = 16
scenario = 1
spwnInterval = 4
randomSeed = 23

cr_method = "DCR"
ctrlPolicy = "DCRControl"
PriorityPolicy ="PriorityScore"
main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed)

cr_method = "TEP"
ctrlPolicy = "TEPControl"
PriorityPolicy ="FCFS"
main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed)

cr_method = "MPIP"
ctrlPolicy = "MPIPControl"
PriorityPolicy ="FCFS"
main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed)

cr_method = "AMPIP"
ctrlPolicy = "AMPIPControl"
PriorityPolicy ="FCFS"
main(cr_method, ctrlPolicy, PriorityPolicy,totalVehicle, scenario, spwnInterval, randomSeed)
