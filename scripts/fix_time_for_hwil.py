import pandas as pd
import numpy as np

orig_path = r".\data\eh6-sustainer\eh6_sus_gps.csv"
save_path = r".\data\eh6-sustainer\eh6_sustainer_hwil_gps.csv"
col = "timestamp"
gps = True
TIME_CONVERSION_FACTOR = 1E6 # timestamp in microseconds
UINT_MAX = 2**32-1

df1 = pd.read_csv(orig_path)
float_time = df1[col]
tmin = float_time.min()
t_shift = 0
if (tmin <= 0):
    t_shift = np.abs(tmin) + 1
new_time = float_time + t_shift # make sure all vals are positve
new_time = new_time * TIME_CONVERSION_FACTOR # convert to us
new_time = new_time.astype(int) # make it integers
assert(new_time.max() <= UINT_MAX) # make sure it won't overflow
df1[col] = new_time
if gps:
    df1.to_csv(save_path, index=False, columns=["timestamp","year","month","day","hour","min","sec","valid_flags","num_sats","lon","lat","height","height_msl","accuracy_horiz","accuracy_vertical","vel_north","vel_east","vel_down","ground_speed","hdg","accuracy_speed","accuracy_hdg"])
else:
    df1.to_csv(save_path, index=False)