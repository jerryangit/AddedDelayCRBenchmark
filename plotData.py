import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import sys
import pandas as pd
import glob
import csv
import os
from matplotlib import rc
from benchmarkMetrics import noConflictAvgDelayMulti
import tikzplotlib

rc('xtick', labelsize=12) 
rc('ytick', labelsize=12) 
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica'],'size'   : 8})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)


def main():
    # 64 vehicles with 4 seconds between spawns, no crossing at the same time, random.seed(23)
    dataPath = r'data/data1/'

    texFigures_dir = r'/home/jerry/Documents/Thesis/5d51a7c02b5c1b4fdd770633/texFigures/'
    data_TEP = {}
    data_MPIP = {}
    data_AMPIP = {}
    data_TDCR = {}
    colors = ['#cb5683','#71a659','#8975ca','#c5783e']
    scenarioList = ['0','1']

    delayAvg = 0.42630768835078786
    maxThroughput = 1/delayAvg

    for scenario in scenarioList:
        if scenario == '0':
            intervalList = ['0.8','1.0','1.2','1.5','2.0','3.0']
        elif scenario == '1':
            intervalList = ['3.2','4.0','4.8','6.0','8.0','12.0']

        for interval in intervalList:
            # Load TEP data
            for f in enumerate(sorted(glob.glob(os.path.join(dataPath, "TEP_fix*_"+scenario+"_"+interval+"_*.csv")))):
                fRead = pd.read_csv(f[1],header = None)
                if f[0] == 0:
                    data_TEP['end_'+scenario+'_'+interval] = [fRead.values[len(fRead)-1][3]/len(fRead.values)]
                    data_TEP['dt_'+scenario+'_'+interval] = []
                else:
                    data_TEP.get('end_'+scenario+'_'+interval).append(fRead.values[len(fRead)-1][3]/len(fRead.values))
                for index,row in fRead.iterrows():
                    data_TEP.get('dt_'+scenario+'_'+interval).append(row.values[3]-row.values[2])
            data_TEP['tot_'+scenario+'_'+interval] = np.average(data_TEP.get('end_'+scenario+'_'+interval))
            data_TEP['avg_'+scenario+'_'+interval] = np.average(data_TEP.get('dt_'+scenario+'_'+interval))
            data_TEP['var_'+scenario+'_'+interval] = np.var(data_TEP.get('dt_'+scenario+'_'+interval))
            
            # Load MPIP data
            for f in enumerate(sorted(glob.glob(os.path.join(dataPath, "MPIP*_"+scenario+"_"+interval+"_*.csv")))):
                fRead = pd.read_csv(f[1],header = None)
                if f[0] == 0:
                    data_MPIP['end_'+scenario+'_'+interval] = [fRead.values[len(fRead)-1][3]/len(fRead.values)]
                    data_MPIP['dt_'+scenario+'_'+interval] = []
                else:
                    data_MPIP.get('end_'+scenario+'_'+interval).append(fRead.values[len(fRead)-1][3]/len(fRead.values))
                for index,row in fRead.iterrows():
                    data_MPIP.get('dt_'+scenario+'_'+interval).append(row.values[3]-row.values[2])
            data_MPIP['tot_'+scenario+'_'+interval] = np.average(data_MPIP.get('end_'+scenario+'_'+interval))
            data_MPIP['avg_'+scenario+'_'+interval] = np.average(data_MPIP.get('dt_'+scenario+'_'+interval))
            data_MPIP['var_'+scenario+'_'+interval] = np.var(data_MPIP.get('dt_'+scenario+'_'+interval))

            # Load AMPIP data
            for f in enumerate(sorted(glob.glob(os.path.join(dataPath, "AMPIP*_"+scenario+"_"+interval+"_*.csv")))):
                fRead = pd.read_csv(f[1],header = None)
                if f[0] == 0:
                    data_AMPIP['end_'+scenario+'_'+interval] = [fRead.values[len(fRead)-1][3]/len(fRead.values)]
                    data_AMPIP['dt_'+scenario+'_'+interval] = []
                else:
                    data_AMPIP.get('end_'+scenario+'_'+interval).append(fRead.values[len(fRead)-1][3]/len(fRead.values))
                for index,row in fRead.iterrows():
                    data_AMPIP.get('dt_'+scenario+'_'+interval).append(row.values[3]-row.values[2])
            data_AMPIP['tot_'+scenario+'_'+interval] = np.average(data_AMPIP.get('end_'+scenario+'_'+interval))            
            data_AMPIP['avg_'+scenario+'_'+interval] = np.average(data_AMPIP.get('dt_'+scenario+'_'+interval))
            data_AMPIP['var_'+scenario+'_'+interval] = np.var(data_AMPIP.get('dt_'+scenario+'_'+interval))

            # Load TDCR data
            for f in enumerate(sorted(glob.glob(os.path.join(dataPath, "DCR*_"+scenario+"_"+interval+"_*.csv")))):
                fRead = pd.read_csv(f[1],header = None)
                if f[0] == 0:
                    data_TDCR['end_'+scenario+'_'+interval] = [fRead.values[len(fRead)-1][3]/len(fRead.values)]
                    data_TDCR['dt_'+scenario+'_'+interval] = []
                else:
                    data_TDCR.get('end_'+scenario+'_'+interval).append(fRead.values[len(fRead)-1][3]/len(fRead.values))
                for index,row in fRead.iterrows():
                    data_TDCR.get('dt_'+scenario+'_'+interval).append(row.values[3]-row.values[2])
            data_TDCR['tot_'+scenario+'_'+interval] = np.average(data_TDCR.get('end_'+scenario+'_'+interval))
            data_TDCR['avg_'+scenario+'_'+interval] = np.average(data_TDCR.get('dt_'+scenario+'_'+interval))
            data_TDCR['var_'+scenario+'_'+interval] = np.var(data_TDCR.get('dt_'+scenario+'_'+interval))


#! Plotting Scenario 0
    conc_0_08 = (data_TEP.get('dt_0_0.8'),data_MPIP.get('dt_0_0.8'),data_AMPIP.get('dt_0_0.8'),data_TDCR.get('dt_0_0.8'))
    conc_0_10 = (data_TEP.get('dt_0_1.0'),data_MPIP.get('dt_0_1.0'),data_AMPIP.get('dt_0_1.0'),data_TDCR.get('dt_0_1.0'))
    conc_0_12 = (data_TEP.get('dt_0_1.2'),data_MPIP.get('dt_0_1.2'),data_AMPIP.get('dt_0_1.2'),data_TDCR.get('dt_0_1.2'))
    conc_0_15 = (data_TEP.get('dt_0_1.5'),data_MPIP.get('dt_0_1.5'),data_AMPIP.get('dt_0_1.5'),data_TDCR.get('dt_0_1.5'))
    conc_0_20 = (data_TEP.get('dt_0_2.0'),data_MPIP.get('dt_0_2.0'),data_AMPIP.get('dt_0_2.0'),data_TDCR.get('dt_0_2.0'))
    conc_0_30 = (data_TEP.get('dt_0_3.0'),data_MPIP.get('dt_0_3.0'),data_AMPIP.get('dt_0_3.0'),data_TDCR.get('dt_0_3.0'))
    overallDelay = {}
    overallDelay['TEP_0'] = np.average([data_TEP.get('tot_0_0.8'),data_TEP.get('tot_0_1.0'),data_TEP.get('tot_0_1.2'),data_TEP.get('tot_0_1.5'),data_TEP.get('tot_0_2.0'),data_TEP.get('tot_0_3.0')])
    overallDelay['MPIP_0'] =  np.average([data_MPIP.get('tot_0_0.8'),data_MPIP.get('tot_0_1.0'),data_MPIP.get('tot_0_1.2'),data_MPIP.get('tot_0_1.5'),data_MPIP.get('tot_0_2.0'),data_MPIP.get('tot_0_3.0')])
    overallDelay['AMPIP_0'] =  np.average([data_AMPIP.get('tot_0_0.8'),data_AMPIP.get('tot_0_1.0'),data_AMPIP.get('tot_0_1.2'),data_AMPIP.get('tot_0_1.5'),data_AMPIP.get('tot_0_2.0'),data_AMPIP.get('tot_0_3.0')])
    overallDelay['TDCR_0'] =  np.average([data_TDCR.get('tot_0_0.8'),data_TDCR.get('tot_0_1.0'),data_TDCR.get('tot_0_1.2'),data_TDCR.get('tot_0_1.5'),data_TDCR.get('tot_0_2.0'),data_TDCR.get('tot_0_3.0')])
#* Fig 0
    fig , axs = plt.subplots(1,2,num=0,sharex=True)
    throughputList = [1.25,1,5/6,2/3,1/2,1/3]
    marginalDelayList = np.array([noConflictAvgDelayMulti(throughputList[i],0,50) for i in range(len(throughputList))])
    axs[0].plot(throughputList,[data_TEP.get('avg_0_0.8'),data_TEP.get('avg_0_1.0'),data_TEP.get('avg_0_1.2'),data_TEP.get('avg_0_1.5'),data_TEP.get('avg_0_2.0'),data_TEP.get('avg_0_3.0')],colors[0])
    axs[0].plot(throughputList,[data_MPIP.get('avg_0_0.8'),data_MPIP.get('avg_0_1.0'),data_MPIP.get('avg_0_1.2'),data_MPIP.get('avg_0_1.5'),data_MPIP.get('avg_0_2.0'),data_MPIP.get('avg_0_3.0')],colors[1])
    axs[0].plot(throughputList,[data_AMPIP.get('avg_0_0.8'),data_AMPIP.get('avg_0_1.0'),data_AMPIP.get('avg_0_1.2'),data_AMPIP.get('avg_0_1.5'),data_AMPIP.get('avg_0_2.0'),data_AMPIP.get('avg_0_3.0')],colors[2])
    axs[0].plot(throughputList,[data_TDCR.get('avg_0_0.8'),data_TDCR.get('avg_0_1.0'),data_TDCR.get('avg_0_1.2'),data_TDCR.get('avg_0_1.5'),data_TDCR.get('avg_0_2.0'),data_TDCR.get('avg_0_3.0')],colors[3])
    axs[1].plot(throughputList,np.array([data_TEP.get('tot_0_0.8'),data_TEP.get('tot_0_1.0'),data_TEP.get('tot_0_1.2'),data_TEP.get('tot_0_1.5'),data_TEP.get('tot_0_2.0'),data_TEP.get('tot_0_3.0')])-marginalDelayList,colors[0])
    axs[1].plot(throughputList,np.array([data_MPIP.get('tot_0_0.8'),data_MPIP.get('tot_0_1.0'),data_MPIP.get('tot_0_1.2'),data_MPIP.get('tot_0_1.5'),data_MPIP.get('tot_0_2.0'),data_MPIP.get('tot_0_3.0')])-marginalDelayList,colors[1])
    axs[1].plot(throughputList,np.array([data_AMPIP.get('tot_0_0.8'),data_AMPIP.get('tot_0_1.0'),data_AMPIP.get('tot_0_1.2'),data_AMPIP.get('tot_0_1.5'),data_AMPIP.get('tot_0_2.0'),data_AMPIP.get('tot_0_3.0')])-marginalDelayList,colors[2])
    axs[1].plot(throughputList,np.array([data_TDCR.get('tot_0_0.8'),data_TDCR.get('tot_0_1.0'),data_TDCR.get('tot_0_1.2'),data_TDCR.get('tot_0_1.5'),data_TDCR.get('tot_0_2.0'),data_TDCR.get('tot_0_3.0')])-marginalDelayList,colors[3])
    axs[1].axhline(delayAvg,color = 'k', linestyle = '--')
    plt.legend(['TEP','MPIP','AMPIP','TDCR'])
    axs[1].set_ylim([0,2.2])
    tikzplotlib.save(texFigures_dir+"travelTime_0_export.tex",axis_width = '8cm', axis_height = '8cm')

#* Fig 1
    fig , axs = plt.subplots(3,1,num=1,sharex=True,sharey=True)
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 60
    binrange = (0.5,60.5)
    axs[2].hist(conc_0_15,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_0_20,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].hist(conc_0_30,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,60)
    axs[2].legend(['TEP,mu=' + str(round(data_TEP.get('avg_0_1.5'),2)) + ', var=' + str(round(data_TEP.get('var_0_1.5'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_0_1.5'),2)) + ', var=' + str(round(data_MPIP.get('var_0_1.5'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_0_1.5'),2)) + ', var=' + str(round(data_AMPIP.get('var_0_1.5'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_0_1.5'),2)) + ', var=' + str(round(data_TDCR.get('var_0_1.5'),2))])
    axs[1].legend(['TEP,mu=' + str(round(data_TEP.get('avg_0_2.0'),2)) + ', var=' + str(round(data_TEP.get('var_0_2.0'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_0_2.0'),2)) + ', var=' + str(round(data_MPIP.get('var_0_2.0'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_0_2.0'),2)) + ', var=' + str(round(data_AMPIP.get('var_0_2.0'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_0_2.0'),2)) + ', var=' + str(round(data_TDCR.get('var_0_2.0'),2))])
    axs[0].legend(['TEP,mu=' + str(round(data_TEP.get('avg_0_3.0'),2)) + ', var=' + str(round(data_TEP.get('var_0_3.0'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_0_3.0'),2)) + ', var=' + str(round(data_MPIP.get('var_0_3.0'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_0_3.0'),2)) + ', var=' + str(round(data_AMPIP.get('var_0_3.0'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_0_3.0'),2)) + ', var=' + str(round(data_TDCR.get('var_0_3.0'),2))])
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_15)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_20)]
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_30)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(13)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)

    tikzplotlib.save(texFigures_dir+"travelTimeHistA_0_export.tex",axis_width = '16cm', axis_height = '5cm')


#* Fig 2
    fig , axs = plt.subplots(3,1,num=2,sharex=True,sharey=True)
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 60
    binrange = (0.5,60.5)
    axs[2].hist(conc_0_08,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_0_10,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].hist(conc_0_12,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,60)
    axs[2].legend(['TEP,mu=' + str(round(data_TEP.get('avg_0_0.8'),2)) + ', var=' + str(round(data_TEP.get('var_0_0.8'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_0_0.8'),2)) + ', var=' + str(round(data_MPIP.get('var_0_0.8'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_0_0.8'),2)) + ', var=' + str(round(data_AMPIP.get('var_0_0.8'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_0_0.8'),2)) + ', var=' + str(round(data_TDCR.get('var_0_0.8'),2))])
    axs[1].legend(['TEP,mu=' + str(round(data_TEP.get('avg_0_1.0'),2)) + ', var=' + str(round(data_TEP.get('var_0_1.0'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_0_1.0'),2)) + ', var=' + str(round(data_MPIP.get('var_0_1.0'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_0_1.0'),2)) + ', var=' + str(round(data_AMPIP.get('var_0_1.0'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_0_1.0'),2)) + ', var=' + str(round(data_TDCR.get('var_0_1.0'),2))])
    axs[0].legend(['TEP,mu=' + str(round(data_TEP.get('avg_0_1.2'),2)) + ', var=' + str(round(data_TEP.get('var_0_1.2'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_0_1.2'),2)) + ', var=' + str(round(data_MPIP.get('var_0_1.2'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_0_1.2'),2)) + ', var=' + str(round(data_AMPIP.get('var_0_1.2'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_0_1.2'),2)) + ', var=' + str(round(data_TDCR.get('var_0_1.2'),2))])
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_08)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_10)]
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_0_12)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(13)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)

    tikzplotlib.save(texFigures_dir+"travelTimeHistB_0_export.tex",axis_width = '16cm', axis_height = '5cm')


#! Plotting Scenario 1
    conc_1_08 = (data_TEP.get('dt_1_3.2'),data_MPIP.get('dt_1_3.2'),data_AMPIP.get('dt_1_3.2'),data_TDCR.get('dt_1_3.2'))
    conc_1_10 = (data_TEP.get('dt_1_4.0'),data_MPIP.get('dt_1_4.0'),data_AMPIP.get('dt_1_4.0'),data_TDCR.get('dt_1_4.0'))
    conc_1_12 = (data_TEP.get('dt_1_4.8'),data_MPIP.get('dt_1_4.8'),data_AMPIP.get('dt_1_4.8'),data_TDCR.get('dt_1_4.8'))
    conc_1_15 = (data_TEP.get('dt_1_6.0'),data_MPIP.get('dt_1_6.0'),data_AMPIP.get('dt_1_6.0'),data_TDCR.get('dt_1_6.0'))
    conc_1_20 = (data_TEP.get('dt_1_8.0'),data_MPIP.get('dt_1_8.0'),data_AMPIP.get('dt_1_8.0'),data_TDCR.get('dt_1_8.0'))
    conc_1_30 = (data_TEP.get('dt_1_12.0'),data_MPIP.get('dt_1_12.0'),data_AMPIP.get('dt_1_12.0'),data_TDCR.get('dt_1_12.0'))
    overallDelay['TEP_1'] = np.average([data_TEP.get('tot_1_3.2'),data_TEP.get('tot_1_4.0'),data_TEP.get('tot_1_4.8'),data_TEP.get('tot_1_6.0'),data_TEP.get('tot_1_8.0'),data_TEP.get('tot_1_12.0')])
    overallDelay['MPIP_1'] =  np.average([data_MPIP.get('tot_1_3.2'),data_MPIP.get('tot_1_4.0'),data_MPIP.get('tot_1_4.8'),data_MPIP.get('tot_1_6.0'),data_MPIP.get('tot_1_8.0'),data_MPIP.get('tot_1_12.0')])
    overallDelay['AMPIP_1'] =  np.average([data_AMPIP.get('tot_1_3.2'),data_AMPIP.get('tot_1_4.0'),data_AMPIP.get('tot_1_4.8'),data_AMPIP.get('tot_1_6.0'),data_AMPIP.get('tot_1_8.0'),data_AMPIP.get('tot_1_12.0')])
    overallDelay['TDCR_1'] =  np.average([data_TDCR.get('tot_1_3.2'),data_TDCR.get('tot_1_4.0'),data_TDCR.get('tot_1_4.8'),data_TDCR.get('tot_1_6.0'),data_TDCR.get('tot_1_8.0'),data_TDCR.get('tot_1_12.0')])

#* Fig 3
    fig , axs = plt.subplots(1,2,num=3,sharex=True)
    marginalDelayList = [noConflictAvgDelayMulti(throughputList[i],1,55) for i in range(len(throughputList))]
    axs[0].plot(throughputList,[data_TEP.get('avg_1_3.2'),data_TEP.get('avg_1_4.0'),data_TEP.get('avg_1_4.8'),data_TEP.get('avg_1_6.0'),data_TEP.get('avg_1_8.0'),data_TEP.get('avg_1_12.0')],colors[0])
    axs[0].plot(throughputList,[data_MPIP.get('avg_1_3.2'),data_MPIP.get('avg_1_4.0'),data_MPIP.get('avg_1_4.8'),data_MPIP.get('avg_1_6.0'),data_MPIP.get('avg_1_8.0'),data_MPIP.get('avg_1_12.0')],colors[1])
    axs[0].plot(throughputList,[data_AMPIP.get('avg_1_3.2'),data_AMPIP.get('avg_1_4.0'),data_AMPIP.get('avg_1_4.8'),data_AMPIP.get('avg_1_6.0'),data_AMPIP.get('avg_1_8.0'),data_AMPIP.get('avg_1_12.0')],colors[2])
    axs[0].plot(throughputList,[data_TDCR.get('avg_1_3.2'),data_TDCR.get('avg_1_4.0'),data_TDCR.get('avg_1_4.8'),data_TDCR.get('avg_1_6.0'),data_TDCR.get('avg_1_8.0'),data_TDCR.get('avg_1_12.0')],colors[3])
    axs[1].plot(throughputList,np.array([data_TEP.get('tot_1_3.2'),data_TEP.get('tot_1_4.0'),data_TEP.get('tot_1_4.8'),data_TEP.get('tot_1_6.0'),data_TEP.get('tot_1_8.0'),data_TEP.get('tot_1_12.0')])-marginalDelayList,colors[0])
    axs[1].plot(throughputList,np.array([data_MPIP.get('tot_1_3.2'),data_MPIP.get('tot_1_4.0'),data_MPIP.get('tot_1_4.8'),data_MPIP.get('tot_1_6.0'),data_MPIP.get('tot_1_8.0'),data_MPIP.get('tot_1_12.0')])-marginalDelayList,colors[1])
    axs[1].plot(throughputList,np.array([data_AMPIP.get('tot_1_3.2'),data_AMPIP.get('tot_1_4.0'),data_AMPIP.get('tot_1_4.8'),data_AMPIP.get('tot_1_6.0'),data_AMPIP.get('tot_1_8.0'),data_AMPIP.get('tot_1_12.0')])-marginalDelayList,colors[2])
    axs[1].plot(throughputList,np.array([data_TDCR.get('tot_1_3.2'),data_TDCR.get('tot_1_4.0'),data_TDCR.get('tot_1_4.8'),data_TDCR.get('tot_1_6.0'),data_TDCR.get('tot_1_8.0'),data_TDCR.get('tot_1_12.0')])-marginalDelayList,colors[3])
    axs[1].axhline(delayAvg,color = 'k', linestyle = '--')
    plt.legend(['TEP','MPIP','AMPIP','TDCR'])
    axs[1].set_ylim([0,2.5])
    tikzplotlib.save(texFigures_dir+"travelTime_1_export.tex",axis_width = '8cm', axis_height = '8cm')


#* Fig 4
    fig , axs = plt.subplots(3,1,num=4,sharex=True,sharey=True)
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 80
    binrange = (0.5,80.5)
    axs[2].hist(conc_1_15,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_1_20,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].hist(conc_1_30,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,80)
    axs[2].legend(['TEP,mu=' + str(round(data_TEP.get('avg_1_6.0'),2)) + ', var=' + str(round(data_TEP.get('var_1_6.0'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_1_6.0'),2)) + ', var=' + str(round(data_MPIP.get('var_1_6.0'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_1_6.0'),2)) + ', var=' + str(round(data_AMPIP.get('var_1_6.0'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_1_6.0'),2)) + ', var=' + str(round(data_TDCR.get('var_1_6.0'),2))])
    axs[1].legend(['TEP,mu=' + str(round(data_TEP.get('avg_1_8.0'),2)) + ', var=' + str(round(data_TEP.get('var_1_8.0'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_1_8.0'),2)) + ', var=' + str(round(data_MPIP.get('var_1_8.0'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_1_8.0'),2)) + ', var=' + str(round(data_AMPIP.get('var_1_8.0'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_1_8.0'),2)) + ', var=' + str(round(data_TDCR.get('var_1_8.0'),2))])
    axs[0].legend(['TEP,mu=' + str(round(data_TEP.get('avg_1_12.0'),2)) + ', var=' + str(round(data_TEP.get('var_1_12.0'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_1_12.0'),2)) + ', var=' + str(round(data_MPIP.get('var_1_12.0'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_1_12.0'),2)) + ', var=' + str(round(data_AMPIP.get('var_1_12.0'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_1_12.0'),2)) + ', var=' + str(round(data_TDCR.get('var_1_12.0'),2))])
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_15)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_20)]
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_30)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(17)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)

    tikzplotlib.save(texFigures_dir+"travelTimeHistA_1_export.tex",axis_width = '16cm', axis_height = '5cm')
    
#* Fig 5
    fig , axs = plt.subplots(3,1,num=5,sharex=True,sharey=True)
    fig.subplots_adjust(wspace=0, hspace=0)
    axs[2].set_xlabel('time (s)')
    n_bins = 80
    binrange = (0.5,80.5)
    axs[2].hist(conc_1_08,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[1].hist(conc_1_10,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].hist(conc_1_12,color = colors, bins = n_bins, range = binrange, density=1,rwidth=1,zorder=1)
    axs[0].set_xlim(0,80)
    axs[2].legend(['TEP,mu=' + str(round(data_TEP.get('avg_1_3.2'),2)) + ', var=' + str(round(data_TEP.get('var_1_3.2'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_1_3.2'),2)) + ', var=' + str(round(data_MPIP.get('var_1_3.2'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_1_3.2'),2)) + ', var=' + str(round(data_AMPIP.get('var_1_3.2'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_1_3.2'),2)) + ', var=' + str(round(data_TDCR.get('var_1_3.2'),2))])
    axs[1].legend(['TEP,mu=' + str(round(data_TEP.get('avg_1_4.0'),2)) + ', var=' + str(round(data_TEP.get('var_1_4.0'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_1_4.0'),2)) + ', var=' + str(round(data_MPIP.get('var_1_4.0'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_1_4.0'),2)) + ', var=' + str(round(data_AMPIP.get('var_1_4.0'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_1_4.0'),2)) + ', var=' + str(round(data_TDCR.get('var_1_4.0'),2))])
    axs[0].legend(['TEP,mu=' + str(round(data_TEP.get('avg_1_4.8'),2)) + ', var=' + str(round(data_TEP.get('var_1_4.8'),2)),'MPIP,mu=' + str(round(data_MPIP.get('avg_1_4.8'),2)) + ', var=' + str(round(data_MPIP.get('var_1_4.8'),2)),'AMPIP,mu=' + str(round(data_AMPIP.get('avg_1_4.8'),2)) + ', var=' + str(round(data_AMPIP.get('var_1_4.8'),2)),'TDCR,mu=' + str(round(data_TDCR.get('avg_1_4.8'),2)) + ', var=' + str(round(data_TDCR.get('var_1_4.8'),2))])
    [axs[2].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_08)]
    [axs[1].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_10)]
    [axs[0].axvline(np.average(_x[1]), color=colors[_x[0]], linestyle='dashed', linewidth=2,zorder=3.5) for _x in enumerate(conc_1_12)]
    for axis in axs:
        axis.set_xticks([i*5 for i in range(17)])
        axis.tick_params(direction='inout',length=10)
        axis.grid(True)

    tikzplotlib.save(texFigures_dir+"travelTimeHistB_1_export.tex",axis_width = '16cm', axis_height = '5cm')


#! AMP differences
    data2Path = r'data/data2/'
    scenarioList = ['1']
    errMarginList = ['0.0', '0.1', '0.2', '0.3', '0.4' ,'0.5' ,'0.6', '0.7', '0.8', '0.9', '1.0']
    errMarginListFloat =  [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]

    for scenario in scenarioList:
        interval = '10.0'
        for errMargin in errMarginList:
            data_AMPIP['dt_'+scenario+'_'+interval+'_'+errMargin] = []
            for f in enumerate(sorted(glob.glob(os.path.join(data2Path, "AMPIP*_"+scenario+"_"+interval+"_*_"+errMargin+"_*.csv")))):
                fRead = pd.read_csv(f[1],header = None)                  
                for index,row in fRead.iterrows():
                    data_AMPIP.get('dt_'+scenario+'_'+interval+'_'+errMargin).append(row.values[3]-row.values[2])
            data_AMPIP['avg_'+scenario+'_'+interval+'_'+errMargin] = np.average(data_AMPIP.get('dt_'+scenario+'_'+interval+'_'+errMargin))
            data_AMPIP['var_'+scenario+'_'+interval+'_'+errMargin] = np.var(data_AMPIP.get('dt_'+scenario+'_'+interval+'_'+errMargin))
#* Fig 6
    fig,ax = plt.subplots(1,1,num=6,sharex=True)
    meanList = [data_AMPIP.get('avg_1_10.0_'+str(i)) for i in errMarginList]
    varList = [data_AMPIP.get('var_1_10.0_'+str(i)) for i in errMarginList]
    ax.plot(errMarginListFloat,meanList,colors[2])
    ax2 = ax.twinx()
    ax2.plot(errMarginListFloat,varList,colors[2],linestyle = '--')
    fig.legend(['Mean','Variance'])
    ax2.set_xlim([0,1])
    ax.set_xlim([0,1])

    tikzplotlib.save(texFigures_dir+"errMarginAMP_export.tex",axis_width = '10cm', axis_height = '6cm')


#! Other calculations etc.
    percentageReduction(overallDelay)

    plt.show()


def percentageReduction(overallDelay):
    print('Scenario 0, MPIP w.r.t TEP ',round(100*(overallDelay.get('TEP_0')-overallDelay.get('MPIP_0'))/overallDelay.get('TEP_0'),2),'%')
    print('Scenario 0, AMPIP w.r.t TEP ',round(100*(overallDelay.get('TEP_0')-overallDelay.get('AMPIP_0'))/overallDelay.get('TEP_0'),2),'%')
    print('Scenario 0, AMPIP w.r.t MPIP ',round(100*(overallDelay.get('MPIP_0')-overallDelay.get('AMPIP_0'))/overallDelay.get('MPIP_0'),2),'%')
    print('Scenario 0, TDCR w.r.t TEP ',round(100*(overallDelay.get('TEP_0')-overallDelay.get('TDCR_0'))/overallDelay.get('TEP_0'),2),'%')
    print('Scenario 0, TDCR w.r.t MPIP ',round(100*(overallDelay.get('MPIP_0')-overallDelay.get('TDCR_0'))/overallDelay.get('MPIP_0'),2),'%')
    print('Scenario 0, TDCR w.r.t AMPIP ',round(100*(overallDelay.get('AMPIP_0')-overallDelay.get('TDCR_0'))/overallDelay.get('AMPIP_0'),2),'%')
    print('Scenario 1, MPIP w.r.t TEP ',round(100*(overallDelay.get('TEP_1')-overallDelay.get('MPIP_1'))/overallDelay.get('TEP_1'),2),'%')
    print('Scenario 1, AMPIP w.r.t TEP ',round(100*(overallDelay.get('TEP_1')-overallDelay.get('AMPIP_1'))/overallDelay.get('TEP_1'),2),'%')
    print('Scenario 1, AMPIP w.r.t MPIP ',round(100*(overallDelay.get('MPIP_1')-overallDelay.get('AMPIP_1'))/overallDelay.get('MPIP_1'),2),'%')
    print('Scenario 1, TDCR w.r.t TEP ',round(100*(overallDelay.get('TEP_1')-overallDelay.get('TDCR_1'))/overallDelay.get('TEP_1'),2),'%')
    print('Scenario 1, TDCR w.r.t MPIP ',round(100*(overallDelay.get('MPIP_1')-overallDelay.get('TDCR_1'))/overallDelay.get('MPIP_1'),2),'%')
    print('Scenario 1, TDCR w.r.t AMPIP ',round(100*(overallDelay.get('AMPIP_1')-overallDelay.get('TDCR_1'))/overallDelay.get('AMPIP_1'),2),'%')
    
if __name__ == '__main__':
    main()
 