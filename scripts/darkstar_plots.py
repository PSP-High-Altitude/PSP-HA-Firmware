import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# from pal_plots import trim_data
TIME_CONVERSION = 1E6

def load_data(file, time_conversion=TIME_CONVERSION):
    df = pd.read_csv(file)
    df['time'] = df['timestamp'] / time_conversion # add converted time
    return df

def trim_data(df, tstart, tstop):
    #TODO: conditions for time not in range
    i_min = np.min(df.index[df['time']>=tstart])
    i_max = np.max(df.index[df['time']<=tstop])
    trimmed_df = df.iloc[i_min:i_max]
    trimmed_df = trimmed_df.reset_index()
    t0 = trimmed_df['time'][0]
    trimmed_df['adjusted_time'] = trimmed_df['time'] - t0
    return trimmed_df

def plot_compare_vertical_se(se_df1, se_df2, names1, names2, xlim=None, name="", legnames=[]):
    if 'adjusted_time' in se_df1: t1 = se_df1['adjusted_time']
    else: t1 = se_df1['time']
    if 'adjusted_time' in se_df2: t2 = se_df2['adjusted_time']
    else: t2 = se_df2['time']
    fig = plt.figure()

    data1 = [se_df1[names1[0]], se_df1[names1[1]], se_df1[names1[2]]]
    data2 = [se_df2[names2[0]], se_df2[names2[1]], se_df2[names2[2]]]
    labels = ["Height (geo) [m]", "Vel (geo) [m/s]", "Acc (body) [m/s^2]"]
    event_idx = se_df1.index[se_df1['flight_phase'].diff() != 0].tolist()
    for p in range(3):
        plt.subplot(3,1,p+1)
        plt.plot(t1, data1[p], lw=1)
        plt.plot(t2, data2[p], lw=1)
        plt.grid(True)
        plt.xlabel("Time (s)")
        plt.ylabel(labels[p])
        if len(legnames) > 0 and p == 1:
            plt.legend(legnames)
        for i in range(len(event_idx)):
            plt.ylim(plt.ylim())
            plt.plot([t1[event_idx[i]],t1[event_idx[i]]], plt.ylim(), '--g', lw=0.6)
            # plt.plot([t1[event_idx[i]],t1[event_idx[i]]], plt.ylim(), '--', color="#700000", lw=0.5)
    plt.suptitle(name)

def plot_compare_accel(se_df1, se_df2, names1, names2, sensor_df, xlim=None, name="", legnames=[]):
    if 'adjusted_time' in se_df1: t1 = se_df1['adjusted_time']
    else: t1 = se_df1['time']
    if 'adjusted_time' in se_df2: t2 = se_df2['adjusted_time']
    else: t2 = se_df2['time']
    if 'adjusted_time' in sensor_df: t3 = sensor_df['adjusted_time']
    else: t3 = sensor_df['time']
    fig = plt.figure()

    data1 = [se_df1[names1[0]], se_df1[names1[1]], se_df1[names1[2]]]
    data2 = [se_df2[names2[0]], se_df2[names2[1]], se_df2[names2[2]]]
    acc_data = [-9.81*(sensor_df["acc_i_z"]+1), -9.81*(sensor_df["acc_h_z"]+1)]
    labels = ["Height (geo) [m]", "Vel (geo) [m/s]", "Acc (body) [m/s^2]"]
    event_idx = se_df1.index[se_df1['flight_phase'].diff() != 0].tolist()
    for p in [2]:
        # plt.subplot(3,1,p+1)
        plt.plot(t1, data1[p], lw=1)
        plt.plot(t2, data2[p], lw=1)
        plt.plot(t3, acc_data[0], ".", ms=0.6)
        plt.plot(t3, acc_data[1], ".", ms=0.6)
        plt.grid(True)
        plt.xlabel("Time (s)")
        plt.ylabel(labels[p])
        if len(legnames) > 0:
            plt.legend(legnames)
        for i in range(len(event_idx)):
            plt.ylim(plt.ylim())
            plt.plot([t1[event_idx[i]],t1[event_idx[i]]], plt.ylim(), '--g', lw=0.6)
            # plt.plot([t1[event_idx[i]],t1[event_idx[i]]], plt.ylim(), '--', color="#700000", lw=0.5)
    plt.suptitle(name)

def plot_diff_vertical_se(se_df1, se_df2, names1, names2, xlim=None, name="", legnames=[]):
    if 'adjusted_time' in se_df1: t1 = se_df1['adjusted_time']
    else: t1 = se_df1['time']
    if 'adjusted_time' in se_df2: t2 = se_df2['adjusted_time']
    else: t2 = se_df2['time']
    fig = plt.figure()

    data1 = [se_df1[names1[0]], se_df1[names1[1]], se_df1[names1[2]]]
    data2 = [se_df2[names2[0]], se_df2[names2[1]], se_df2[names2[2]]]
    vars1 = [se_df1['pos_var_ekf'], se_df1['vel_var_ekf'], se_df1['acc_var_ekf']]
    labels = ["Height (geo) [m]", "Vel (geo) [m/s]", "Acc (body) [m/s^2]"]
    event_idx = se_df1.index[se_df1['flight_phase'].diff() != 0].tolist()
    for p in range(3):
        plt.subplot(3,1,p+1)
        plt.plot(t1, data1[p]-data2[p], lw=1)
        plt.plot(t1, np.sqrt(vars1[p])*3, '--r', lw=.9)
        plt.plot(t1, -np.sqrt(vars1[p])*3, '--r', lw=.9)
        # plt.plot(t2, data2[p], lw=1)
        plt.grid(True)
        plt.xlabel("Time (s)")
        plt.ylabel(labels[p])
        if len(legnames) > 0 and p == 1:
            plt.legend(legnames)
        for i in range(len(event_idx)):
            plt.ylim(plt.ylim())
            plt.plot([t1[event_idx[i]],t1[event_idx[i]]], plt.ylim(), '--g', lw=0.7)
            # plt.plot([t1[event_idx[i]],t1[event_idx[i]]], plt.ylim(), '--', color="#700000", lw=0.5)
    plt.suptitle(name)

def plot_altitude_pressure(se_df, data_df, names, refnames, name):
    if 'adjusted_time' in se_df: t = se_df['adjusted_time']
    else: t = se_df['time']
    if 'adjusted_time' in data_df: t_data = data_df['adjusted_time']
    else: t_data = data_df['time']
    fig = plt.figure()

    data = [se_df[names[0]], se_df[names[0]], se_df[names[1]]]
    data2 = [se_df[refnames[0]], se_df[refnames[0]], se_df[refnames[1]]]
    vars = [se_df['pos_var_ekf'], se_df['pos_var_ekf'], se_df['vel_var_ekf']]
    labels = ["Height (geo) [m]", "Height (geo) [m]", "Vel (body) [m/s]"]
    event_idx = se_df.index[se_df['flight_phase'].diff() != 0].tolist()
    for p in range(3):
        ax1 = plt.subplot(3,1,p+1)
        ax1.plot(t, data[p], lw=1)
        ax1.grid(True)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel(labels[p])
        if (p == 0):
            ax2 = ax1.twinx()  
            ax2.plot(t_data, data_df['pressure'], 'g', lw=.9)
            ax2.set_ylabel("pressure (mbar)")

            h0 = pressureToAlt(data_df['pressure'][0])
            ax1.plot(t_data, pressureToAlt(data_df['pressure'])-h0, 'r.', alpha=1, ms=.09)

            ax1.legend(["kf height", "pressure", "pressure height"])
        else:
            ax1.plot(t, data2[p], lw=1)
            ax2 = ax1.twinx()  
            ax2.plot(t, np.sqrt(vars[p])*3, '--r', lw=.9)
            ax2.set_ylabel("3 stdev")
            ax1.legend(["kf", "state est", "stdev"])

        for i in range(len(event_idx)):
            ax1.set_ylim(ax1.get_ylim())
            ax1.plot([t[event_idx[i]],t[event_idx[i]]], ax1.get_ylim(), 'm--', lw=0.6)
    plt.suptitle(name)
    
    

def plot_kf_se(se_df, xlim=None, name=""):
    if 'adjusted_time' in se_df: t = se_df['adjusted_time']
    else: t = se_df['time']
    fig = plt.figure()

    data = [se_df['pos_ekf'], se_df['vel_ekf'], se_df['acc_ekf']]
    vars = [se_df['pos_var_ekf'], se_df['vel_var_ekf'], se_df['acc_var_ekf']]
    labels = ["Height (geo) [m]", "Vel (geo) [m/s]", "Acc (body) [m/s^2]"]
    event_idx = se_df.index[se_df['flight_phase'].diff() != 0].tolist()
    for p in range(3):
        ax1 = plt.subplot(3,1,p+1)
        ax1.plot(t, data[p], lw=1)
        ax1.grid(True)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel(labels[p])
        ax2 = ax1.twinx()  
        ax2.plot(t, np.sqrt(vars[p])*3, '--r', lw=.9)
        ax2.set_ylabel("3 stdev")

        for i in range(len(event_idx)):
            ax1.set_ylim(ax1.get_ylim())
            ax1.plot([t[event_idx[i]],t[event_idx[i]]], ax1.get_ylim(), 'g--', lw=0.6)
    plt.suptitle(name)

def pressureToAlt(pressure):
    SEA_LEVEL_PRESSURE = 1013.25  # Standard sea level pressure in millibars
    LAPSE_RATE = 0.0065           # Standard temperature lapse rate in K/m
    return 44330 * (1 - np.power((pressure / SEA_LEVEL_PRESSURE), 1 / 5.25588))

def plot_gps_state(gps_df, xlim=None, name=""):
    if 'adjusted_time' in gps_df: t = gps_df['adjusted_time']
    else: t = gps_df['time']
    fig = plt.figure()

    plt.subplot(2,1,1)
    y = gps_df['height']
    plt.plot(t, y, label="GPS Height")
    bound = np.array(gps_df.loc[:,'accuracy_vertical'])
    bound[gps_df.index[gps_df.loc[:,'fix_valid'] == 0]] = np.nan
    plt.plot(t, y+bound, ':', t, y-bound, ':', color='#666666', lw=.5)
    plt.fill_between(t, y-bound, y+bound, facecolor='#ffff00', alpha=0.3, label="Accuracy Bound")
    plt.grid(True)
    plt.xlabel("time (s)")
    plt.ylabel("GPS Height (m)")

    plt.subplot(2,1,2)
    y = gps_df['vel_down'] * -1
    plt.plot(t, y, label='GPS Speed')
    bound = np.array(gps_df.loc[:,'accuracy_speed'])
    bound[gps_df.index[gps_df.loc[:,'fix_valid'] == 0]] = np.nan
    plt.plot(t, y+bound, ':', t, y-bound, ':', color='#666666', lw=.5)
    plt.fill_between(t, y-bound, y+bound, facecolor='#ffff00', alpha=0.3, label="Accuracy Bound")
    plt.grid(True)
    plt.xlabel("time (s)")
    plt.ylabel("GPS Speed (m/s)")
    # return fig