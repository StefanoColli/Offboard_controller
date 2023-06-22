#! /usr/bin/python

from pyulog.core import ULog
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import pandas as pd
import numpy as np
import sys
import os

def Plot_3D_trajectory(ax, df): 

    ax.plot(df['pos_x'], df['pos_y'], df['pos_z'],
            label="Followed path", linewidth=0.7)
    ax.plot(df['stp_x'], df['stp_y'], df['stp_z'],
            label="Reference path", linestyle=(0,(1,1)))

    # Axes settings
    ax.set_xticks(np.arange(round(np.nanmin(df['stp_x'])),
                             round(np.nanmax(df['stp_x'])+1), 1))
    ax.set_yticks(np.arange(round(np.nanmin(df['stp_y'])),
                             round(np.nanmax(df['stp_y'])+1), 1))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Drone 3D Trajectory")
    ax.legend(title="Paths")

def Plot_2D_trajectory(ax, df, dimension):
    ax.plot(df['pos_t'], df['pos_' + dimension],
            label="Followed path", linewidth=0.7)
    ax.plot(df['stp_t'], df[ 'stp_' + dimension],
            label="Reference path", linestyle=(0,(1,1)))          

    ax.set_xlabel("t [s]")
    ax.set_ylabel(dimension + " [m]")
    ax.set_title(dimension + " Trajectory")
    #ax.legend(title="Paths")

def RMSE(df):

    return ((df['pos_x'] - df['stp_x']) ** 2 + \
           (df['pos_y'] - df['stp_y']) ** 2 + \
           (df['pos_z'] - df['stp_z']) ** 2 ).mean() ** 0.5
    
def ISE(df):

    return ((df['pos_x'] - df['stp_x']) ** 2 + \
           (df['pos_y'] - df['stp_y']) ** 2 + \
           (df['pos_z'] - df['stp_z']) ** 2 ).sum()
        
#open csv file and retrieve data
if len(sys.argv) < 2:
    print("WARNING: No input csv file specified, trying to use out.csv...")
    csv_file = "./out.csv"
else:
    csv_file = sys.argv[1]

if not os.path.isfile(csv_file):
    print("ERROR: can't find " + csv_file)
    exit(-1)

#### Dataframe building and data preprocessing ####

df = pd.read_csv(csv_file)

df['pos_z'] = df['pos_z'] * -1 #account for NED frame
df['stp_z'] = df['stp_z'] * -1 #account for NED frame

df['pos_t'] = (df['pos_t'] - df['pos_t'][0]) / 1000 #convert us timestamps in trajectory time [s]
df['stp_t'] = (df['stp_t'] - df['stp_t'][0]) / 1000 #convert us timestamps in trajectory time [s] 

#### Compute RMSE and ISE ####
rmse = RMSE(df)
ise = ISE(df)

#### Plotting ####

#create figure with 3x3 grid
fig = plt.figure(csv_file + " Analisys")
gs = fig.add_gridspec(3,3)

#3D plot of the trajectory
ax_3D = fig.add_subplot(gs[:,:-1] ,projection='3d')
Plot_3D_trajectory(ax_3D, df)

#Little textbox with rmse and ise
text = f"RMSE = {rmse:.4f}\nISE = {ise:.4f}" 
fig.text(x=0.03, y=0.05,s=text , fontsize = 18 , bbox = dict(facecolor = 'lightskyblue', alpha = 0.5))

#Plots of the single dimensions
dimensions = ['x', 'y', 'z']
for i in range(3):
    ax = fig.add_subplot(gs[i,2])
    Plot_2D_trajectory(ax, df, dimensions[i])

#Finishing touches
plt.tight_layout(pad=.1) #TODO too close to right border?
figManager = plt.get_current_fig_manager()
figManager.window.showMaximized() #display maximized
plt.show()