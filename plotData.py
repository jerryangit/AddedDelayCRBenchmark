import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import glob
import csv
import os

def main():
    # 64 vehicles with 4 seconds between spawns, no crossing at the same time, random.seed(23)
    dataPath = r'data/Run1/'
    #! Scenario 1
    # 0.5 interval
    data_TEP_0_05 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_0_0.5_*.csv"))))
    data_MPIP_0_05 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_0_0.5_*.csv"))))
    data_AMPIP_0_05 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_0_0.5_*.csv"))))
    data_DCR_0_05 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_0_0.5_*.csv"))))
    # 1.0 interval
    data_TEP_0_10 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_0_1.0_*.csv"))))
    data_MPIP_0_10 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_0_1.0_*.csv"))))
    data_AMPIP_0_10 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_0_1.0_*.csv"))))
    data_DCR_0_10 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_0_1.0_*.csv"))))
    # 1.2 interval
    data_TEP_0_12 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_0_1.2_*.csv"))))
    data_MPIP_0_12 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_0_1.2_*.csv"))))
    data_AMPIP_0_12 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_0_1.2_*.csv"))))
    data_DCR_0_12 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_0_1.2_*.csv"))))
    # 1.5 interval
    data_TEP_0_15 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_0_1.5_*.csv"))))
    data_MPIP_0_15 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_0_1.5_*.csv"))))
    data_AMPIP_0_15 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_0_1.5_*.csv"))))
    data_DCR_0_15 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_0_1.5_*.csv"))))
    # 2.0 interval
    data_TEP_0_20 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_0_2.0_*.csv"))))
    data_MPIP_0_20 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_0_2.0_*.csv"))))
    data_AMPIP_0_20 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_0_2.0_*.csv"))))
    data_DCR_0_20 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_0_2.0_*.csv"))))
    # 3.0 interval
    data_TEP_0_30 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_0_3.0_*.csv"))))
    data_MPIP_0_30 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_0_3.0_*.csv"))))
    data_AMPIP_0_30 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_0_3.0_*.csv"))))
    data_DCR_0_30 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_0_3.0_*.csv"))))

    #! Scenario 1
    # 2.0 interval
    data_TEP_1_20 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_1_2.0_*.csv"))))
    data_MPIP_1_20 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_1_2.0_*.csv"))))
    data_AMPIP_1_20 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_1_2.0_*.csv"))))
    data_DCR_1_20 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_1_2.0_*.csv"))))
    # 4.0 interval
    data_TEP_1_40 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_1_4.0_*.csv"))))
    data_MPIP_1_40 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_1_4.0_*.csv"))))
    data_AMPIP_1_40 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_1_4.0_*.csv"))))
    data_DCR_1_40 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_1_4.0_*.csv"))))
    # 4.8 interval
    data_TEP_1_48 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_1_4.8_*.csv"))))
    data_MPIP_1_48 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_1_4.8_*.csv"))))
    data_AMPIP_1_48 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_1_4.8_*.csv"))))
    data_DCR_1_48 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_1_4.8_*.csv"))))
    # 6.0 interval
    data_TEP_1_60 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_1_6.0_*.csv"))))
    data_MPIP_1_60 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_1_6.0_*.csv"))))
    data_AMPIP_1_60 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_1_6.0_*.csv"))))
    data_DCR_1_60 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_1_6.0_*.csv"))))
    # 8.0 interval
    data_TEP_1_80 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_1_8.0_*.csv"))))
    data_MPIP_1_80 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_1_8.0_*.csv"))))
    data_AMPIP_1_80 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_1_8.0_*.csv"))))
    data_DCR_1_80 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_1_8.0_*.csv"))))
    # 12.0 interval
    data_TEP_1_120 =  pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "TEP_fix*_1_12.0_*.csv"))))
    data_MPIP_1_120 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "MPIP*_1_12.0_*.csv"))))
    data_AMPIP_1_120 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "AMPIP*_1_12.0_*.csv"))))
    data_DCR_1_120 = pd.concat((pd.read_csv(f,header = None) for f in glob.glob(os.path.join(dataPath, "DCR*_1_12.0_*.csv"))))

    #! Plotting
    # TEP delays
    data_TEP_0_05_dt = []
    data_TEP_0_10_dt = []
    data_TEP_0_12_dt = []
    data_TEP_0_15_dt = []
    data_TEP_0_20_dt = []
    data_TEP_0_30_dt = []

    for index,row in data_TEP_0_05.iterrows():
        if index > 0:
            data_TEP_0_05_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_10.iterrows():
        if index > 0:
            data_TEP_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_12.iterrows():
        if index > 0:
            data_TEP_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_15.iterrows():
        if index > 0:
            data_TEP_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_20.iterrows():
        if index > 0:
            data_TEP_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_30.iterrows():
        if index > 0:
            data_TEP_0_30_dt.append(row.values[3]-row.values[2])
            
    data_TEP_0_05_avg = np.average(data_TEP_0_05_dt)
    data_TEP_0_10_avg = np.average(data_TEP_0_10_dt)
    data_TEP_0_12_avg = np.average(data_TEP_0_12_dt)
    data_TEP_0_15_avg = np.average(data_TEP_0_15_dt)
    data_TEP_0_20_avg = np.average(data_TEP_0_20_dt)
    data_TEP_0_30_avg = np.average(data_TEP_0_30_dt)

    # MPIP delays
    data_MPIP_0_05_dt = []
    data_MPIP_0_10_dt = []
    data_MPIP_0_12_dt = []
    data_MPIP_0_15_dt = []
    data_MPIP_0_20_dt = []
    data_MPIP_0_30_dt = []

    for index,row in data_MPIP_0_05.iterrows():
        if index > 0:
            data_MPIP_0_05_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_10.iterrows():
        if index > 0:
            data_MPIP_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_12.iterrows():
        if index > 0:
            data_MPIP_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_15.iterrows():
        if index > 0:
            data_MPIP_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_20.iterrows():
        if index > 0:
            data_MPIP_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_30.iterrows():
        if index > 0:
            data_MPIP_0_30_dt.append(row.values[3]-row.values[2])
            
    data_MPIP_0_05_avg = np.average(data_MPIP_0_05_dt)
    data_MPIP_0_10_avg = np.average(data_MPIP_0_10_dt)
    data_MPIP_0_12_avg = np.average(data_MPIP_0_12_dt)
    data_MPIP_0_15_avg = np.average(data_MPIP_0_15_dt)
    data_MPIP_0_20_avg = np.average(data_MPIP_0_20_dt)
    data_MPIP_0_30_avg = np.average(data_MPIP_0_30_dt)

    # AMPIP delays
    data_AMPIP_0_05_dt = []
    data_AMPIP_0_10_dt = []
    data_AMPIP_0_12_dt = []
    data_AMPIP_0_15_dt = []
    data_AMPIP_0_20_dt = []
    data_AMPIP_0_30_dt = []

    for index,row in data_AMPIP_0_05.iterrows():
        if index > 0:
            data_AMPIP_0_05_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_10.iterrows():
        if index > 0:
            data_AMPIP_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_12.iterrows():
        if index > 0:
            data_AMPIP_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_15.iterrows():
        if index > 0:
            data_AMPIP_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_20.iterrows():
        if index > 0:
            data_AMPIP_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_30.iterrows():
        if index > 0:
            data_AMPIP_0_30_dt.append(row.values[3]-row.values[2])
            
    data_AMPIP_0_05_avg = np.average(data_AMPIP_0_05_dt)
    data_AMPIP_0_10_avg = np.average(data_AMPIP_0_10_dt)
    data_AMPIP_0_12_avg = np.average(data_AMPIP_0_12_dt)
    data_AMPIP_0_15_avg = np.average(data_AMPIP_0_15_dt)
    data_AMPIP_0_20_avg = np.average(data_AMPIP_0_20_dt)
    data_AMPIP_0_30_avg = np.average(data_AMPIP_0_30_dt)

    # DCR delays
    data_DCR_0_05_dt = []
    data_DCR_0_10_dt = []
    data_DCR_0_12_dt = []
    data_DCR_0_15_dt = []
    data_DCR_0_20_dt = []
    data_DCR_0_30_dt = []

    for index,row in data_DCR_0_05.iterrows():
        if index > 0:
            data_DCR_0_05_dt.append(row.values[3]-row.values[2])
    for index,row in data_DCR_0_10.iterrows():
        if index > 0:
            data_DCR_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_DCR_0_12.iterrows():
        if index > 0:
            data_DCR_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_DCR_0_15.iterrows():
        if index > 0:
            data_DCR_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_DCR_0_20.iterrows():
        if index > 0:
            data_DCR_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_DCR_0_30.iterrows():
        if index > 0:
            data_DCR_0_30_dt.append(row.values[3]-row.values[2])
            
    data_DCR_0_05_avg = np.average(data_DCR_0_05_dt)
    data_DCR_0_10_avg = np.average(data_DCR_0_10_dt)
    data_DCR_0_12_avg = np.average(data_DCR_0_12_dt)
    data_DCR_0_15_avg = np.average(data_DCR_0_15_dt)
    data_DCR_0_20_avg = np.average(data_DCR_0_20_dt)
    data_DCR_0_30_avg = np.average(data_DCR_0_30_dt)

    colors = ['r','g','b','c']

    plt.figure(0)
    plt.plot([2,1,5/6,2/3,1/2,1/3],[data_TEP_0_05_avg,data_TEP_0_10_avg,data_TEP_0_12_avg,data_TEP_0_15_avg,data_TEP_0_20_avg,data_TEP_0_30_avg],'r')
    plt.plot([2,1,5/6,2/3,1/2,1/3],[data_MPIP_0_05_avg,data_MPIP_0_10_avg,data_MPIP_0_12_avg,data_MPIP_0_15_avg,data_MPIP_0_20_avg,data_MPIP_0_30_avg],'g')
    plt.plot([2,1,5/6,2/3,1/2,1/3],[data_AMPIP_0_05_avg,data_AMPIP_0_10_avg,data_AMPIP_0_12_avg,data_AMPIP_0_15_avg,data_AMPIP_0_20_avg,data_AMPIP_0_30_avg],'b')
    plt.plot([2,1,5/6,2/3,1/2,1/3],[data_DCR_0_05_avg,data_DCR_0_10_avg,data_DCR_0_12_avg,data_DCR_0_15_avg,data_DCR_0_20_avg,data_DCR_0_30_avg],'c')

    plt.title("Mean Travel Time (s) vs Throughput (1/Spawn Interval)")
    plt.legend(['TEP','MPIP','AMPIP','DCR'])

    fig , axs = plt.subplots(3,1,num=1)
    n_bins = 60
    binrange = (0,60)
    axs[0].hist((data_TEP_0_05_dt,data_MPIP_0_05_dt,data_AMPIP_0_05_dt,data_DCR_0_05_dt),color = colors, bins = n_bins, range = binrange)
    axs[1].hist((data_TEP_0_10_dt,data_MPIP_0_10_dt,data_AMPIP_0_10_dt,data_DCR_0_10_dt),color = colors, bins = n_bins, range = binrange)
    axs[2].hist((data_TEP_0_12_dt,data_MPIP_0_12_dt,data_AMPIP_0_12_dt,data_DCR_0_12_dt),color = colors, bins = n_bins, range = binrange)
    #plt.legend(['TEP','MPIP','AMPIP','DCR'])
    axs[0].set_xlim(0,60)
    axs[1].set_xlim(0,60)
    axs[2].set_xlim(0,60)
    axs[0].set_title("Travel Time Histogram 2 per second")
    axs[1].set_title("Travel Time Histogram 1 per second")
    axs[2].set_title("Travel Time Histogram 5/6 per second")

    fig , axs = plt.subplots(3,1,num=2)
    n_bins = 60
    binrange = (0,60)
    axs[0].hist((data_TEP_0_15_dt,data_MPIP_0_15_dt,data_AMPIP_0_15_dt,data_DCR_0_15_dt),color = colors, bins = n_bins, range = binrange)
    axs[1].hist((data_TEP_0_20_dt,data_MPIP_0_20_dt,data_AMPIP_0_20_dt,data_DCR_0_20_dt),color = colors, bins = n_bins, range = binrange)
    axs[2].hist((data_TEP_0_30_dt,data_MPIP_0_30_dt,data_AMPIP_0_30_dt,data_DCR_0_30_dt),color = colors, bins = n_bins, range = binrange)
    #plt.legend(['TEP','MPIP','AMPIP','DCR'])
    axs[0].set_xlim(0,60)
    axs[1].set_xlim(0,60)
    axs[2].set_xlim(0,60)
    axs[0].set_title("Travel Time Histogram 2/3 per second")
    axs[1].set_title("Travel Time Histogram 1/2 per second")
    axs[2].set_title("Travel Time Histogram 1/3 per second")


    plt.show()
    

    plt.figure(0)
    plt.title("Mean Travel Time (s) vs Throughput (1/Spawn Interval)")

    plt.figure(0)
    plt.title("Mean Travel Time (s) vs Throughput (1/Spawn Interval)")



def old():
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