import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import sys
import pandas as pd
import glob
import csv
import os
from matplotlib import rc
import tikzplotlib

rc('xtick', labelsize=12) 
rc('ytick', labelsize=12) 
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica'],'size'   : 8})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)

def main():
    # 64 vehicles with 4 seconds between spawns, no crossing at the same time, random.seed(23)
    dataPath = r'data/Run3/'
    dataPathB = r'data/Run5/'

    #! Scenario 1
    # 0.8 interval
    data_TEP_0_08 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_0_0.8_*.csv")))))
    data_MPIP_0_08 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_0_0.8_*.csv")))))
    data_AMPIP_0_08 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_0_0.8_*.csv")))))
    data_TDCR_0_08 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_0_0.8_*.csv")))))
    # 1.0 interval
    data_TEP_0_10 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_0_1.0_*.csv")))))
    data_MPIP_0_10 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_0_1.0_*.csv")))))
    data_AMPIP_0_10 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_0_1.0_*.csv")))))
    data_TDCR_0_10 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_0_1.0_*.csv")))))
    # 1.2 interval
    data_TEP_0_12 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_0_1.2_*.csv")))))
    data_MPIP_0_12 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_0_1.2_*.csv")))))
    data_AMPIP_0_12 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_0_1.2_*.csv")))))
    data_TDCR_0_12 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_0_1.2_*.csv")))))
    # 1.5 interval
    data_TEP_0_15 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_0_1.5_*.csv")))))
    data_MPIP_0_15 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_0_1.5_*.csv")))))
    data_AMPIP_0_15 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_0_1.5_*.csv")))))
    data_TDCR_0_15 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_0_1.5_*.csv")))))
    # 2.0 interval
    data_TEP_0_20 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_0_2.0_*.csv")))))
    data_MPIP_0_20 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_0_2.0_*.csv")))))
    data_AMPIP_0_20 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_0_2.0_*.csv")))))
    data_TDCR_0_20 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_0_2.0_*.csv")))))
    # 3.0 interval
    data_TEP_0_30 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_0_3.0_*.csv")))))
    data_MPIP_0_30 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_0_3.0_*.csv")))))
    data_AMPIP_0_30 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_0_3.0_*.csv")))))
    data_TDCR_0_30 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_0_3.0_*.csv")))))

    #! Scenario 1
    # 3.2 interval
    data_TEP_1_32 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_1_3.2_*.csv")))))
    data_MPIP_1_32 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_1_3.2_*.csv")))))
    data_AMPIP_1_32 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_1_3.2_*.csv")))))
    data_TDCR_1_32 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_1_3.2_*.csv")))))
    # 4.0 interval
    data_TEP_1_40 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_1_4.0_*.csv")))))
    data_MPIP_1_40 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_1_4.0_*.csv")))))
    data_AMPIP_1_40 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_1_4.0_*.csv")))))
    data_TDCR_1_40 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_1_4.0_*.csv")))))
    # 4.8 interval
    data_TEP_1_48 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_1_4.8_*.csv")))))
    data_MPIP_1_48 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_1_4.8_*.csv")))))
    data_AMPIP_1_48 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_1_4.8_*.csv")))))
    data_TDCR_1_48 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_1_4.8_*.csv")))))
    # 6.0 interval
    data_TEP_1_60 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_1_6.0_*.csv")))))
    data_MPIP_1_60 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_1_6.0_*.csv")))))
    data_AMPIP_1_60 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_1_6.0_*.csv")))))
    data_TDCR_1_60 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_1_6.0_*.csv")))))
    # 8.0 interval
    data_TEP_1_80 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_1_8.0_*.csv")))))
    data_MPIP_1_80 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_1_8.0_*.csv")))))
    data_AMPIP_1_80 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_1_8.0_*.csv")))))
    data_TDCR_1_80 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_1_8.0_*.csv")))))
    # 12.0 interval
    data_TEP_1_120 =  pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_1_12.0_*.csv")))))
    data_MPIP_1_120 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "MPIP*_1_12.0_*.csv")))))
    data_AMPIP_1_120 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "AMPIP*_1_12.0_*.csv")))))
    data_TDCR_1_120 = pd.concat((pd.read_csv(f,header = None) for f in sorted(glob.glob(os.path.join(dataPath, "DCR*_1_12.0_*.csv")))))

    #! Plotting Scenario 0
    # TEP delays
    data_TEP_0_08_dt = []
    data_TEP_0_10_dt = []
    data_TEP_0_12_dt = []
    data_TEP_0_15_dt = []
    data_TEP_0_20_dt = []
    data_TEP_0_30_dt = []

    for index,row in data_TEP_0_08.iterrows():
        data_TEP_0_08_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_10.iterrows():
        data_TEP_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_12.iterrows():
        data_TEP_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_15.iterrows():
        data_TEP_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_20.iterrows():
        data_TEP_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_0_30.iterrows():
        data_TEP_0_30_dt.append(row.values[3]-row.values[2])
            
    data_TEP_0_08_avg = np.average(data_TEP_0_08_dt)
    data_TEP_0_10_avg = np.average(data_TEP_0_10_dt)
    data_TEP_0_12_avg = np.average(data_TEP_0_12_dt)
    data_TEP_0_15_avg = np.average(data_TEP_0_15_dt)
    data_TEP_0_20_avg = np.average(data_TEP_0_20_dt)
    data_TEP_0_30_avg = np.average(data_TEP_0_30_dt)
    data_TEP_0_08_var = np.var(data_TEP_0_08_dt)
    data_TEP_0_10_var = np.var(data_TEP_0_10_dt)
    data_TEP_0_12_var = np.var(data_TEP_0_12_dt)
    data_TEP_0_15_var = np.var(data_TEP_0_15_dt)
    data_TEP_0_20_var = np.var(data_TEP_0_20_dt)
    data_TEP_0_30_var = np.var(data_TEP_0_30_dt)

    # MPIP delays
    data_MPIP_0_08_dt = []
    data_MPIP_0_10_dt = []
    data_MPIP_0_12_dt = []
    data_MPIP_0_15_dt = []
    data_MPIP_0_20_dt = []
    data_MPIP_0_30_dt = []

    for index,row in data_MPIP_0_08.iterrows():
        data_MPIP_0_08_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_10.iterrows():
        data_MPIP_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_12.iterrows():
        data_MPIP_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_15.iterrows():
        data_MPIP_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_20.iterrows():
        data_MPIP_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_0_30.iterrows():
        data_MPIP_0_30_dt.append(row.values[3]-row.values[2])
            
    data_MPIP_0_08_avg = np.average(data_MPIP_0_08_dt)
    data_MPIP_0_10_avg = np.average(data_MPIP_0_10_dt)
    data_MPIP_0_12_avg = np.average(data_MPIP_0_12_dt)
    data_MPIP_0_15_avg = np.average(data_MPIP_0_15_dt)
    data_MPIP_0_20_avg = np.average(data_MPIP_0_20_dt)
    data_MPIP_0_30_avg = np.average(data_MPIP_0_30_dt)
    data_MPIP_0_08_var = np.var(data_MPIP_0_08_dt)
    data_MPIP_0_10_var = np.var(data_MPIP_0_10_dt)
    data_MPIP_0_12_var = np.var(data_MPIP_0_12_dt)
    data_MPIP_0_15_var = np.var(data_MPIP_0_15_dt)
    data_MPIP_0_20_var = np.var(data_MPIP_0_20_dt)
    data_MPIP_0_30_var = np.var(data_MPIP_0_30_dt)

    # AMPIP delays
    data_AMPIP_0_08_dt = []
    data_AMPIP_0_10_dt = []
    data_AMPIP_0_12_dt = []
    data_AMPIP_0_15_dt = []
    data_AMPIP_0_20_dt = []
    data_AMPIP_0_30_dt = []

    for index,row in data_AMPIP_0_08.iterrows():
        data_AMPIP_0_08_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_10.iterrows():
        data_AMPIP_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_12.iterrows():
        data_AMPIP_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_15.iterrows():
        data_AMPIP_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_20.iterrows():
        data_AMPIP_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_0_30.iterrows():
        data_AMPIP_0_30_dt.append(row.values[3]-row.values[2])
            
    data_AMPIP_0_08_avg = np.average(data_AMPIP_0_08_dt)
    data_AMPIP_0_10_avg = np.average(data_AMPIP_0_10_dt)
    data_AMPIP_0_12_avg = np.average(data_AMPIP_0_12_dt)
    data_AMPIP_0_15_avg = np.average(data_AMPIP_0_15_dt)
    data_AMPIP_0_20_avg = np.average(data_AMPIP_0_20_dt)
    data_AMPIP_0_30_avg = np.average(data_AMPIP_0_30_dt)
    data_AMPIP_0_08_var = np.var(data_AMPIP_0_08_dt)
    data_AMPIP_0_10_var = np.var(data_AMPIP_0_10_dt)
    data_AMPIP_0_12_var = np.var(data_AMPIP_0_12_dt)
    data_AMPIP_0_15_var = np.var(data_AMPIP_0_15_dt)
    data_AMPIP_0_20_var = np.var(data_AMPIP_0_20_dt)
    data_AMPIP_0_30_var = np.var(data_AMPIP_0_30_dt)


    # TDCR delays
    data_TDCR_0_08_dt = []
    data_TDCR_0_10_dt = []
    data_TDCR_0_12_dt = []
    data_TDCR_0_15_dt = []
    data_TDCR_0_20_dt = []
    data_TDCR_0_30_dt = []

    for index,row in data_TDCR_0_08.iterrows():
        data_TDCR_0_08_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_0_10.iterrows():
        data_TDCR_0_10_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_0_12.iterrows():
        data_TDCR_0_12_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_0_15.iterrows():
        data_TDCR_0_15_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_0_20.iterrows():
        data_TDCR_0_20_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_0_30.iterrows():
        data_TDCR_0_30_dt.append(row.values[3]-row.values[2])
            
    data_TDCR_0_08_avg = np.average(data_TDCR_0_08_dt)
    data_TDCR_0_10_avg = np.average(data_TDCR_0_10_dt)
    data_TDCR_0_12_avg = np.average(data_TDCR_0_12_dt)
    data_TDCR_0_15_avg = np.average(data_TDCR_0_15_dt)
    data_TDCR_0_20_avg = np.average(data_TDCR_0_20_dt)
    data_TDCR_0_30_avg = np.average(data_TDCR_0_30_dt)
    data_TDCR_0_08_var = np.var(data_TDCR_0_08_dt)
    data_TDCR_0_10_var = np.var(data_TDCR_0_10_dt)
    data_TDCR_0_12_var = np.var(data_TDCR_0_12_dt)
    data_TDCR_0_15_var = np.var(data_TDCR_0_15_dt)
    data_TDCR_0_20_var = np.var(data_TDCR_0_20_dt)
    data_TDCR_0_30_var = np.var(data_TDCR_0_30_dt)

    colors = ['#cb5683','#71a659','#8975ca','#c5783e']

    plt.figure(0)
    plt.tight_layout()
    plt.plot([1.25,1,5/6,2/3,1/2,1/3],[data_TEP_0_08_avg,data_TEP_0_10_avg,data_TEP_0_12_avg,data_TEP_0_15_avg,data_TEP_0_20_avg,data_TEP_0_30_avg],colors[0])
    plt.plot([1.25,1,5/6,2/3,1/2,1/3],[data_MPIP_0_08_avg,data_MPIP_0_10_avg,data_MPIP_0_12_avg,data_MPIP_0_15_avg,data_MPIP_0_20_avg,data_MPIP_0_30_avg],colors[1])
    plt.plot([1.25,1,5/6,2/3,1/2,1/3],[data_AMPIP_0_08_avg,data_AMPIP_0_10_avg,data_AMPIP_0_12_avg,data_AMPIP_0_15_avg,data_AMPIP_0_20_avg,data_AMPIP_0_30_avg],colors[2])
    plt.plot([1.25,1,5/6,2/3,1/2,1/3],[data_TDCR_0_08_avg,data_TDCR_0_10_avg,data_TDCR_0_12_avg,data_TDCR_0_15_avg,data_TDCR_0_20_avg,data_TDCR_0_30_avg],colors[3])

    # plt.title("Mean Travel Time (s) vs Throughput (1/Spawn Interval), Random Arrivals")
    plt.legend(['TEP','MPIP','AMPIP','TDCR'])

    texFigures_dir = r'/home/jerry/Documents/Thesis/5d51a7c02b5c1b4fdd770633/texFigures/'
    tikzplotlib.save(texFigures_dir+"travelTime_0_export.tex",axis_width = '16cm', axis_height = '8cm')

    conc_0_08 = (data_TEP_0_08_dt,data_MPIP_0_08_dt,data_AMPIP_0_08_dt,data_TDCR_0_08_dt)
    conc_0_10 = (data_TEP_0_10_dt,data_MPIP_0_10_dt,data_AMPIP_0_10_dt,data_TDCR_0_10_dt) 
    conc_0_12 = (data_TEP_0_12_dt,data_MPIP_0_12_dt,data_AMPIP_0_12_dt,data_TDCR_0_12_dt)
    conc_0_15 = (data_TEP_0_15_dt,data_MPIP_0_15_dt,data_AMPIP_0_15_dt,data_TDCR_0_15_dt)
    conc_0_20 = (data_TEP_0_20_dt,data_MPIP_0_20_dt,data_AMPIP_0_20_dt,data_TDCR_0_20_dt)
    conc_0_30 = (data_TEP_0_30_dt,data_MPIP_0_30_dt,data_AMPIP_0_30_dt,data_TDCR_0_30_dt)
    
    # plt.figure(3)
    # plt.title('0.08')
    # plt.plot(data_MPIP_0_08_dt,'r')
    # plt.plot(data_AMPIP_0_08_dt,'b')
    # plt.show()

    # plt.title('0.10')
    # plt.plot(data_MPIP_0_10_dt,'r')
    # plt.plot(data_AMPIP_0_10_dt,'b')
    # plt.show()

    # plt.title('0.12')
    # plt.plot(data_MPIP_0_12_dt,'r')
    # plt.plot(data_AMPIP_0_12_dt,'b')
    # plt.show()

    # plt.title('0.15')
    # plt.plot(data_MPIP_0_15_dt,'r')
    # plt.plot(data_AMPIP_0_15_dt,'b')
    # plt.show()


    fig , axs = plt.subplots(3,1,num=1,sharex=True,sharey=True)
    fig.tight_layout()
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 60
    binrange = (0.5,60.5)
    axs[0].hist(conc_0_08,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_0_10,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[2].hist(conc_0_12,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,60)
    axs[0].legend(['TEP,mu='+str(round(data_TEP_0_08_avg,2))+'var='+str(round(data_TEP_0_08_var,2)),'MPIP,mu='+str(round(data_MPIP_0_08_avg,2))+'var='+str(round(data_MPIP_0_08_var,2)),'AMPIP,mu='+str(round(data_AMPIP_0_08_avg,2))+'var='+str(round(data_AMPIP_0_08_var,2)),'TDCR,mu='+str(round(data_TDCR_0_08_avg,2))+'var='+str(round(data_TDCR_0_08_var,2))],loc="upper right")
    axs[1].legend(['TEP,mu='+str(round(data_TEP_0_10_avg,2))+'var='+str(round(data_TEP_0_10_var,2)),'MPIP,mu='+str(round(data_MPIP_0_10_avg,2))+'var='+str(round(data_MPIP_0_10_var,2)),'AMPIP,mu='+str(round(data_AMPIP_0_10_avg,2))+'var='+str(round(data_AMPIP_0_10_var,2)),'TDCR,mu='+str(round(data_TDCR_0_10_avg,2))+'var='+str(round(data_TDCR_0_10_var,2))],loc="upper right")
    axs[2].legend(['TEP,mu='+str(round(data_TEP_0_12_avg,2))+'var='+str(round(data_TEP_0_12_var,2)),'MPIP,mu='+str(round(data_MPIP_0_12_avg,2))+'var='+str(round(data_MPIP_0_12_var,2)),'AMPIP,mu='+str(round(data_AMPIP_0_12_avg,2))+'var='+str(round(data_AMPIP_0_12_var,2)),'TDCR,mu='+str(round(data_TDCR_0_12_avg,2))+'var='+str(round(data_TDCR_0_12_var,2))],loc="upper right")
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_08)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_10)]
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_12)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(13)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)



    tikzplotlib.save(texFigures_dir+"travelTimeHistA_0_export.tex",axis_width = '16cm', axis_height = '5cm')


    fig , axs = plt.subplots(3,1,num=2,sharex=True,sharey=True)
    fig.tight_layout()
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 60
    binrange = (0.5,60.5)
    axs[0].hist(conc_0_15,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_0_20,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[2].hist(conc_0_30,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,60)
    axs[0].legend(['TEP,mu='+str(round(data_TEP_0_15_avg,2))+'var='+str(round(data_TEP_0_15_var,2)),'MPIP,mu='+str(round(data_MPIP_0_15_avg,2))+'var='+str(round(data_MPIP_0_15_var,2)),'AMPIP,mu='+str(round(data_AMPIP_0_15_avg,2))+'var='+str(round(data_AMPIP_0_15_var,2)),'TDCR,mu='+str(round(data_TDCR_0_15_avg,2))+'var='+str(round(data_TDCR_0_15_var,2))],loc="upper right")
    axs[1].legend(['TEP,mu='+str(round(data_TEP_0_20_avg,2))+'var='+str(round(data_TEP_0_20_var,2)),'MPIP,mu='+str(round(data_MPIP_0_20_avg,2))+'var='+str(round(data_MPIP_0_20_var,2)),'AMPIP,mu='+str(round(data_AMPIP_0_20_avg,2))+'var='+str(round(data_AMPIP_0_20_var,2)),'TDCR,mu='+str(round(data_TDCR_0_20_avg,2))+'var='+str(round(data_TDCR_0_20_var,2))],loc="upper right")
    axs[2].legend(['TEP,mu='+str(round(data_TEP_0_30_avg,2))+'var='+str(round(data_TEP_0_30_var,2)),'MPIP,mu='+str(round(data_MPIP_0_30_avg,2))+'var='+str(round(data_MPIP_0_30_var,2)),'AMPIP,mu='+str(round(data_AMPIP_0_30_avg,2))+'var='+str(round(data_AMPIP_0_30_var,2)),'TDCR,mu='+str(round(data_TDCR_0_30_avg,2))+'var='+str(round(data_TDCR_0_30_var,2))],loc="upper right")
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_15)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_20)]
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_30)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(13)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)

    tikzplotlib.save(texFigures_dir+"travelTimeHistB_0_export.tex",axis_width = '16cm', axis_height = '5cm')


    #! Plotting Scenario 1
    # TEP delays
    data_TEP_1_32_dt = []
    data_TEP_1_40_dt = []
    data_TEP_1_48_dt = []
    data_TEP_1_60_dt = []
    data_TEP_1_80_dt = []
    data_TEP_1_120_dt = []

    for index,row in data_TEP_1_32.iterrows():
     
        data_TEP_1_32_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_1_40.iterrows():
     
        data_TEP_1_40_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_1_48.iterrows():
     
        data_TEP_1_48_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_1_60.iterrows():
     
        data_TEP_1_60_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_1_80.iterrows():
     
        data_TEP_1_80_dt.append(row.values[3]-row.values[2])
    for index,row in data_TEP_1_120.iterrows():
     
        data_TEP_1_120_dt.append(row.values[3]-row.values[2])
            
    data_TEP_1_32_avg = np.average(data_TEP_1_32_dt)
    data_TEP_1_40_avg = np.average(data_TEP_1_40_dt)
    data_TEP_1_48_avg = np.average(data_TEP_1_48_dt)
    data_TEP_1_60_avg = np.average(data_TEP_1_60_dt)
    data_TEP_1_80_avg = np.average(data_TEP_1_80_dt)
    data_TEP_1_120_avg = np.average(data_TEP_1_120_dt)
    data_TEP_1_32_var = np.var(data_TEP_1_32_dt)
    data_TEP_1_40_var = np.var(data_TEP_1_40_dt)
    data_TEP_1_48_var = np.var(data_TEP_1_48_dt)
    data_TEP_1_60_var = np.var(data_TEP_1_60_dt)
    data_TEP_1_80_var = np.var(data_TEP_1_80_dt)
    data_TEP_1_120_var = np.var(data_TEP_1_120_dt)

    # MPIP delays
    data_MPIP_1_32_dt = []
    data_MPIP_1_40_dt = []
    data_MPIP_1_48_dt = []
    data_MPIP_1_60_dt = []
    data_MPIP_1_80_dt = []
    data_MPIP_1_120_dt = []

    for index,row in data_MPIP_1_32.iterrows():
     
        data_MPIP_1_32_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_1_40.iterrows():
     
        data_MPIP_1_40_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_1_48.iterrows():
     
        data_MPIP_1_48_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_1_60.iterrows():
     
        data_MPIP_1_60_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_1_80.iterrows():
     
        data_MPIP_1_80_dt.append(row.values[3]-row.values[2])
    for index,row in data_MPIP_1_120.iterrows():
     
        data_MPIP_1_120_dt.append(row.values[3]-row.values[2])
            
    data_MPIP_1_32_avg = np.average(data_MPIP_1_32_dt)
    data_MPIP_1_40_avg = np.average(data_MPIP_1_40_dt)
    data_MPIP_1_48_avg = np.average(data_MPIP_1_48_dt)
    data_MPIP_1_60_avg = np.average(data_MPIP_1_60_dt)
    data_MPIP_1_80_avg = np.average(data_MPIP_1_80_dt)
    data_MPIP_1_120_avg = np.average(data_MPIP_1_120_dt)
    data_MPIP_1_32_var = np.var(data_MPIP_1_32_dt)
    data_MPIP_1_40_var = np.var(data_MPIP_1_40_dt)
    data_MPIP_1_48_var = np.var(data_MPIP_1_48_dt)
    data_MPIP_1_60_var = np.var(data_MPIP_1_60_dt)
    data_MPIP_1_80_var = np.var(data_MPIP_1_80_dt)
    data_MPIP_1_120_var = np.var(data_MPIP_1_120_dt)

    # AMPIP delays
    data_AMPIP_1_32_dt = []
    data_AMPIP_1_40_dt = []
    data_AMPIP_1_48_dt = []
    data_AMPIP_1_60_dt = []
    data_AMPIP_1_80_dt = []
    data_AMPIP_1_120_dt = []

    for index,row in data_AMPIP_1_32.iterrows():
     
        data_AMPIP_1_32_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_1_40.iterrows():
     
        data_AMPIP_1_40_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_1_48.iterrows():
     
        data_AMPIP_1_48_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_1_60.iterrows():
     
        data_AMPIP_1_60_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_1_80.iterrows():
     
        data_AMPIP_1_80_dt.append(row.values[3]-row.values[2])
    for index,row in data_AMPIP_1_120.iterrows():
     
        data_AMPIP_1_120_dt.append(row.values[3]-row.values[2])
            
    data_AMPIP_1_32_avg = np.average(data_AMPIP_1_32_dt)
    data_AMPIP_1_40_avg = np.average(data_AMPIP_1_40_dt)
    data_AMPIP_1_48_avg = np.average(data_AMPIP_1_48_dt)
    data_AMPIP_1_60_avg = np.average(data_AMPIP_1_60_dt)
    data_AMPIP_1_80_avg = np.average(data_AMPIP_1_80_dt)
    data_AMPIP_1_120_avg = np.average(data_AMPIP_1_120_dt)
    data_AMPIP_1_32_var = np.var(data_AMPIP_1_32_dt)
    data_AMPIP_1_40_var = np.var(data_AMPIP_1_40_dt)
    data_AMPIP_1_48_var = np.var(data_AMPIP_1_48_dt)
    data_AMPIP_1_60_var = np.var(data_AMPIP_1_60_dt)
    data_AMPIP_1_80_var = np.var(data_AMPIP_1_80_dt)
    data_AMPIP_1_120_var = np.var(data_AMPIP_1_120_dt)
    
    # TDCR delays
    data_TDCR_1_32_dt = []
    data_TDCR_1_40_dt = []
    data_TDCR_1_48_dt = []
    data_TDCR_1_60_dt = []
    data_TDCR_1_80_dt = []
    data_TDCR_1_120_dt = []

    for index,row in data_TDCR_1_32.iterrows():
     
        data_TDCR_1_32_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_1_40.iterrows():
     
        data_TDCR_1_40_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_1_48.iterrows():
     
        data_TDCR_1_48_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_1_60.iterrows():
     
        data_TDCR_1_60_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_1_80.iterrows():
     
        data_TDCR_1_80_dt.append(row.values[3]-row.values[2])
    for index,row in data_TDCR_1_120.iterrows():
     
        data_TDCR_1_120_dt.append(row.values[3]-row.values[2])
            
    data_TDCR_1_32_avg = np.average(data_TDCR_1_32_dt)
    data_TDCR_1_40_avg = np.average(data_TDCR_1_40_dt)
    data_TDCR_1_48_avg = np.average(data_TDCR_1_48_dt)
    data_TDCR_1_60_avg = np.average(data_TDCR_1_60_dt)
    data_TDCR_1_80_avg = np.average(data_TDCR_1_80_dt)
    data_TDCR_1_120_avg = np.average(data_TDCR_1_120_dt)
    data_TDCR_1_32_var = np.var(data_TDCR_1_32_dt)
    data_TDCR_1_40_var = np.var(data_TDCR_1_40_dt)
    data_TDCR_1_48_var = np.var(data_TDCR_1_48_dt)
    data_TDCR_1_60_var = np.var(data_TDCR_1_60_dt)
    data_TDCR_1_80_var = np.var(data_TDCR_1_80_dt)
    data_TDCR_1_120_var = np.var(data_TDCR_1_120_dt)
    
    plt.figure(3)
    plt.tight_layout()

    plt.plot([1.2,1,5/6,2/3,1/2,1/3],[data_TEP_1_32_avg,data_TEP_1_40_avg,data_TEP_1_48_avg,data_TEP_1_60_avg,data_TEP_1_80_avg,data_TEP_1_120_avg],colors[0])
    plt.plot([1.2,1,5/6,2/3,1/2,1/3],[data_MPIP_1_32_avg,data_MPIP_1_40_avg,data_MPIP_1_48_avg,data_MPIP_1_60_avg,data_MPIP_1_80_avg,data_MPIP_1_120_avg],colors[1])
    plt.plot([1.2,1,5/6,2/3,1/2,1/3],[data_AMPIP_1_32_avg,data_AMPIP_1_40_avg,data_AMPIP_1_48_avg,data_AMPIP_1_60_avg,data_AMPIP_1_80_avg,data_AMPIP_1_120_avg],colors[2])
    plt.plot([1.2,1,5/6,2/3,1/2,1/3],[data_TDCR_1_32_avg,data_TDCR_1_40_avg,data_TDCR_1_48_avg,data_TDCR_1_60_avg,data_TDCR_1_80_avg,data_TDCR_1_120_avg],colors[3])

    plt.title("Mean Travel Time (s) vs Throughput (1/Spawn Interval), Simultaneous Arrivals")
    plt.legend(['TEP','MPIP','AMPIP','TDCR'])

    tikzplotlib.save(texFigures_dir+"travelTime_1_export.tex",axis_width = '16cm', axis_height = '8cm')



    conc_1_32 = (data_TEP_1_32_dt,data_MPIP_1_32_dt,data_AMPIP_1_32_dt,data_TDCR_1_32_dt)
    conc_1_40 = (data_TEP_1_40_dt,data_MPIP_1_40_dt,data_AMPIP_1_40_dt,data_TDCR_1_40_dt) 
    conc_1_48 = (data_TEP_1_48_dt,data_MPIP_1_48_dt,data_AMPIP_1_48_dt,data_TDCR_1_48_dt)
    conc_1_60 = (data_TEP_1_60_dt,data_MPIP_1_60_dt,data_AMPIP_1_60_dt,data_TDCR_1_60_dt)
    conc_1_80 = (data_TEP_1_80_dt,data_MPIP_1_80_dt,data_AMPIP_1_80_dt,data_TDCR_1_80_dt)
    conc_1_120 = (data_TEP_1_120_dt,data_MPIP_1_120_dt,data_AMPIP_1_120_dt,data_TDCR_1_120_dt)


    fig , axs = plt.subplots(3,1,num=4,sharex=True,sharey=True)
    fig.tight_layout()
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 80
    binrange = (0.5,80.5)
    axs[0].hist(conc_1_32,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_1_40,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[2].hist(conc_1_48,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,60)
    axs[0].legend(['TEP,mu='+str(round(data_TEP_1_32_avg,2))+'var='+str(round(data_TEP_1_32_var,2)),'MPIP,mu='+str(round(data_MPIP_1_32_avg,2))+'var='+str(round(data_MPIP_1_32_var,2)),'AMPIP,mu='+str(round(data_AMPIP_1_32_avg,2))+'var='+str(round(data_AMPIP_1_32_var,2)),'TDCR,mu='+str(round(data_TDCR_1_32_avg,2))+'var='+str(round(data_TDCR_1_32_var,2))],loc="upper right")
    axs[1].legend(['TEP,mu='+str(round(data_TEP_1_40_avg,2))+'var='+str(round(data_TEP_1_40_var,2)),'MPIP,mu='+str(round(data_MPIP_1_40_avg,2))+'var='+str(round(data_MPIP_1_40_var,2)),'AMPIP,mu='+str(round(data_AMPIP_1_40_avg,2))+'var='+str(round(data_AMPIP_1_40_var,2)),'TDCR,mu='+str(round(data_TDCR_1_40_avg,2))+'var='+str(round(data_TDCR_1_40_var,2))],loc="upper right")
    axs[2].legend(['TEP,mu='+str(round(data_TEP_1_48_avg,2))+'var='+str(round(data_TEP_1_48_var,2)),'MPIP,mu='+str(round(data_MPIP_1_48_avg,2))+'var='+str(round(data_MPIP_1_48_var,2)),'AMPIP,mu='+str(round(data_AMPIP_1_48_avg,2))+'var='+str(round(data_AMPIP_1_48_var,2)),'TDCR,mu='+str(round(data_TDCR_1_48_avg,2))+'var='+str(round(data_TDCR_1_48_var,2))],loc="upper right")
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_32)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_40)]
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_48)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(17)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)

    tikzplotlib.save(texFigures_dir+"travelTimeHistA_1_export.tex",axis_width = '16cm', axis_height = '5cm')



    fig , axs = plt.subplots(3,1,num=5,sharex=True,sharey=True)
    fig.tight_layout()
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 80
    binrange = (0.5,80.5)
    axs[0].hist(conc_1_60,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_1_80,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[2].hist(conc_1_120,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,60)
    axs[0].legend(['TEP,mu='+str(round(data_TEP_1_60_avg,2))+'var='+str(round(data_TEP_1_60_var,2)),'MPIP,mu='+str(round(data_MPIP_1_60_avg,2))+'var='+str(round(data_MPIP_1_60_var,2)),'AMPIP,mu='+str(round(data_AMPIP_1_60_avg,2))+'var='+str(round(data_AMPIP_1_60_var,2)),'TDCR,mu='+str(round(data_TDCR_1_60_avg,2))+'var='+str(round(data_TDCR_1_60_var,2))],loc="upper right")
    axs[1].legend(['TEP,mu='+str(round(data_TEP_1_80_avg,2))+'var='+str(round(data_TEP_1_80_var,2)),'MPIP,mu='+str(round(data_MPIP_1_80_avg,2))+'var='+str(round(data_MPIP_1_80_var,2)),'AMPIP,mu='+str(round(data_AMPIP_1_80_avg,2))+'var='+str(round(data_AMPIP_1_80_var,2)),'TDCR,mu='+str(round(data_TDCR_1_80_avg,2))+'var='+str(round(data_TDCR_1_80_var,2))],loc="upper right")
    axs[2].legend(['TEP,mu='+str(round(data_TEP_1_120_avg,2))+'var='+str(round(data_TEP_1_120_var,2)),'MPIP,mu='+str(round(data_MPIP_1_120_avg,2))+'var='+str(round(data_MPIP_1_120_var,2)),'AMPIP,mu='+str(round(data_AMPIP_1_120_avg,2))+'var='+str(round(data_AMPIP_1_120_var,2)),'TDCR,mu='+str(round(data_TDCR_1_120_avg,2))+'var='+str(round(data_TDCR_1_120_var,2))],loc="upper right")
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_60)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_80)]
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_120)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(17)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)

    tikzplotlib.save(texFigures_dir+"travelTimeHistB_1_export.tex",axis_width = '16cm', axis_height = '5cm')


    plt.show()


if __name__ == '__main__':
    main()
