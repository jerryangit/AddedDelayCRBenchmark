import matplotlib.pyplot as plt
import numpy as np
import csv


def main():
    # 64 vehicles with 4 seconds between spawns, no crossing at the same time, random.seed(23)
    data_nodelay_enter = []
    data_nodelay_exit = []
    data_nodelay_tIn = []
    data_nodelay_tOut = []
    data_nodelay_tDelta = []
    with open('data/data_nodelay.csv') as data_nodelay_file:
        data_nodelay_reader = csv.reader(data_nodelay_file, delimiter=',',quoting=csv.QUOTE_NONNUMERIC)
        for row in data_nodelay_reader:
            data_nodelay_enter.append(int(row[0]))
            data_nodelay_exit.append(int(row[1]))
            data_nodelay_tIn.append(float(row[2]))
            data_nodelay_tOut.append(float(row[3]))
            data_nodelay_tDelta.append(float(row[3])-float(row[2]))
    plt.figure(0)
    plt.title("Delay (s) for 64 vehicles with 4 seconds between spawns, no crossing at the same time")
    plt.hist(data_nodelay_tDelta)
   
    # 64 vehicles with 1 seconds between random spawns, MP-IP, random.seed(23)
    data_1_enter = []
    data_1_exit = []
    data_1_tIn = []
    data_1_tOut = []
    data_1_tDelta = []
    with open('data/data_1.csv') as data_1_file:
        data_1_reader = csv.reader(data_1_file, delimiter=',',quoting=csv.QUOTE_NONNUMERIC)
        for row in data_1_reader:
            data_1_enter.append(float(row[0]))
            data_1_exit.append(float(row[1]))
            data_1_tIn.append(float(row[2]))
            data_1_tOut.append(float(row[3]))
            data_1_tDelta.append(float(row[3])-float(row[2]))
    plt.figure(1)
    plt.title("Delay (s) for 64 vehicles with 1 seconds between random spawns, [MP-IP]")
    plt.hist(data_1_tDelta)
    
    # 64 vehicles with 4 seconds between simultaneous spawns, MP-IP, random.seed(23)
    data_2_enter = []
    data_2_exit = []
    data_2_tIn = []
    data_2_tOut = []
    data_2_tDelta = []
    with open('data/data_2.csv') as data_2_file:
        data_2_reader = csv.reader(data_2_file, delimiter=',',quoting=csv.QUOTE_NONNUMERIC)
        for row in data_2_reader:
            data_2_enter.append(float(row[0]))
            data_2_exit.append(float(row[1]))
            data_2_tIn.append(float(row[2]))
            data_2_tOut.append(float(row[3]))
            data_2_tDelta.append(float(row[3])-float(row[2]))
    plt.figure(2)
    plt.title("Delay (s) for 64 vehicles with 4 seconds between simultaneous spawns, [MP-IP]")
    plt.hist(data_2_tDelta)
    

    plt.show()

    

if __name__ == '__main__':
    main()