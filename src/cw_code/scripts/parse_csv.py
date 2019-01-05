#!usrbinenv python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import matplotlib as mtplt

from scipy import stats

import pprint
import os
import argparse
import csv
import pprint

#-- Use IMU sample files plot standard deviation across frequency sweep
#-- Output optimum frequency with the lowest standard deviation




# Returns a dictionary of csv filename keywords paired with 100x3 arrays of related acceleration data.

def return_csv_dict(rltv_fldr_pth):

    #-- relative path to folder storing .csv files from static experiment --#
    #rltv_fldr_pth = "./csv_data/IMU_ID_1994/static_f_sweep_x_axis/"

    #-- List all files in the "folder" specified by the input args --#
    files = os.listdir(rltv_fldr_pth)


    # dictionary where key is the csv filename, and the value is the corresponding 100 x 3 array of data.
    experiment_dict = {}


    #-- Loop through files in folder --#
    for filename in files:

        #pprint.pprint(filename)

        with open(rltv_fldr_pth + filename, "r") as csv_file:
            
            # -- Split rows of CSV file
            csv_reader = csv.reader(csv_file, delimiter=',')
            
            data_array_100_by_3 = []

            for row in csv_reader:
                #pprint.pprint(row)
                data_array_100_by_3.append(row[1:]) #append all useful data collumns (exclude the first)

            # Append a new Key:Value pair to the dictionary
            experiment_dict[filename] = data_array_100_by_3

    return experiment_dict


def generate_stats_plots(fldr_pth):

    my_dict = return_csv_dict(fldr_pth)

    mu_x = []
    mu_y = []
    mu_z = []

    sigmas_x = []
    sigmas_y = []
    sigmas_z = []

    for file in my_dict.items():

        x = []
        y = []
        z = []

        #-- Extract x,y,z accelerations from individual csv files --#
        for row in file[1][1:]:
            x.append(float(row[0]))
            y.append(float(row[1]))
            z.append(float(row[2]))


        #-- Order data --#
        x_sorted = sorted(x)
        y_sorted = sorted(y)
        z_sorted = sorted(z)
        #-- Gather statistical info from data --#
        mu_x.append(np.mean(x_sorted))
        mu_y.append(np.mean(y_sorted))
        mu_z.append(np.mean(z_sorted))
        #-- Add particular std-dev to running list --#
        sigmas_x.append(np.std(x_sorted))
        sigmas_y.append(np.std(y_sorted))
        sigmas_z.append(np.std(z_sorted))


    #-- Frequency values sewpt through in the experiment --#
    step = 40 # step can be inferred from the csv file
    f_sweep = list(range(4,1000,step)) 

    #-- Plot for each axis within same window (regardless of axis of interest) [rows,col,index]--#

    mtplt.rcParams["axes.formatter.useoffset"] = False

    #-- x-axis --#
    plt.subplot(311)

    plt.xlabel("$freq$" + " " + "$(Hz)$", fontsize = 15)
    plt.ylabel(r'$g_\mu$' + " " +"$(m/s^2)$", fontsize = 15)
    plt.title("x-axis", fontsize = 25)

    plt.margins(x=0.1, y=0.05) 
    
    sigmas_x = np.array(sigmas_x)
    plt.errorbar(f_sweep, mu_x, sigmas_x, linestyle = "None", marker = ".", markersize = 20, markerfacecolor = 'r' )

    #-- y-axis --#
    plt.subplot(312)

    plt.xlabel("$freq$" + " " + "$(Hz)$", fontsize = 15)
    plt.ylabel(r'$g_\mu$' + " " +"$(m/s^2)$", fontsize = 15)
    plt.title("y-axis", fontsize = 25)

    plt.margins(x=0.1, y=0.05) 

    # y_formatter = ticker.ScalarFormatter(useOffset=False)
    # ax.yaxis.set_major_formatter(y_formatter)

    sigmas_y = np.array(sigmas_y)
    plt.errorbar(f_sweep, mu_y, sigmas_y, linestyle = "None", marker = ".", markersize = 20, markerfacecolor = 'r' )

    



    #-- z-axis --#
    plt.subplot(313)

    plt.xlabel("$freq$" + " " + "$(Hz)$", fontsize = 15)
    plt.ylabel(r'$g_\mu$' + " " +"$(m/s^2)$", fontsize = 15)
    plt.title("z-axis", fontsize = 25)

    plt.margins(x=0.1, y=0.05) 

    sigmas_z = np.array(sigmas_z)
    plt.errorbar(f_sweep, mu_z, sigmas_z, linestyle = "None", marker = ".", markersize = 20, markerfacecolor = 'r' )

    plt.subplots_adjust(hspace=0.6)


    plt.show()
    #plt.savefig("./graphs/" + fldr_pth.split("/")[-2] + ".png", dpi=300)
    #print("../../../graphs/" + fldr_pth.split("/")[-2] + ".png")
def main():
    try:
 
        #-- Plot standard deviation plots for each static test (one for each axis parallel to gravity vector) --#
        generate_stats_plots("./csv_data/IMU_ID_1994/static_f_sweep_x_axis/")
        generate_stats_plots("./csv_data/IMU_ID_1994/static_f_sweep_y_axis/")
        generate_stats_plots("./csv_data/IMU_ID_1994/static_f_sweep_z_axis/")
        #plt.show()

    #-- React to keyboard Interrupt --#
    except KeyboardInterrupt:
        print("nKeyboard Interrupt pressed.")
        shutdown_handler()
        return 0

if __name__ == '__main__':
    main()