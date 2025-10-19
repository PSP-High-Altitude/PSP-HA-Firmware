# Import('env')
import sys
# sys.path.append(r'C:\Users\hkadl\OneDrive - purdue.edu\Documents\PSP\Flight-Analysis\PAL Post Flight')
from darkstar_plots import *

# from pal_plots import load_sensor_data, plot_pressure, plot_pressure_alt
# from darkstar_plots import *

# c_out_file = r'C:\Users\hkadl\OneDrive - purdue.edu\Documents\PSP\PSP-HA-Firmware\lib\control\kf_out.csv'
# data_in_file2 = r'C:\Users\hkadl\OneDrive - purdue.edu\Documents\PSP\Python\Data Files\pal_test\pal_fsl_test_dat.csv'
# data_in_file = r'C:\Users\hkadl\OneDrive - purdue.edu\Documents\PSP\Python\Data Files\pal_test\dat_trimmed.csv'
# data_in_file = r'C:\Users\hkadl\OneDrive - purdue.edu\Documents\PSP\PSP-HA-Firmware\sim_out\sensor.csv'
 # kf_out_file = r'C:\Users\hkadl\OneDrive - purdue.edu\Documents\PSP\PSP-HA-Firmware\sim_out\state.csv'
# NEW_NAMES = ["timestamp",	"flight_phase",	"pos_geo_x",	"pos_geo_y",	"pos_geo_z",	"vel_geo_x",	"vel_geo_y",	"vel_geo_z",	"acc_geo_x",	"acc_geo_y",    "acc_geo_z", 	"pos_geo_x",	"pos_geo_y",	"pos_geo_z",	"vel_geo_x",	"vel_geo_y",	"vel_geo_z",	"acc_geo_x",	"acc_geo_y",	"acc_geo_z",    "angvel_body_x",	"angvel_body_y",	"angvel_body_z",	"orient_geo_w",	"orient_geo_x",	"orient_geo_y",	"orient_geo_z"]

data_in_file = r'.\sim_out\sensor.csv'
kf_out_file = r'.\sim_out\state.csv'



data = load_data(data_in_file)
data = trim_data(data, 25, 40)

kf_state = load_data(kf_out_file)
kf_state = trim_data(kf_state, 25, 40) # trim to ascent
vert_state_names = ['pos_vert', 'vel_vert', 'acc_vert']
kf_names = ['pos_ekf', 'vel_ekf', 'acc_ekf']
# plot_pressure(data)
# plot_kf_se(kf_state, name="kf state")
name = "SWIL EH2 boost no pressure" # TODO: name as command line input
# xl1 = [25,45]
plot_compare_vertical_se(kf_state, kf_state, kf_names, vert_state_names, name=name, legnames=["kf state", "state"])
# plot_altitude_pressure(kf_state, data, kf_names, vert_state_names, name)
# plot_kf_se(kf_state, name=name)
plot_diff_vertical_se(kf_state, kf_state, kf_names, vert_state_names, name=name, legnames=["kf state", "state"])

# plot error

# accel compare
plot_compare_accel(kf_state, kf_state, kf_names, vert_state_names, data, name=name, legnames=["kf state", "state", "acc_i", "acc_h"])

plt.show()
plt.pause(.1) # make plot not disappear in debug mode

# TODO: Trim to match range
