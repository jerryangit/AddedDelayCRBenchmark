from main import main
import random
random.seed(0)
totalVehicle = 256
scenario = 0
spwnInterval = 1.5
logging = 0
randomSeed = random.randint(0,1000000) # Generate randoms seed between 0 and 100000,logging0

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


scenario = 1
spwnInterval = 6

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
