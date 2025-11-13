import pandas as pd
import numpy as np

"""
the old flights don't have z as the vertical axis, 
and they used to work but now they don't (Om changed something) 
so this will reorient them so I can use them again
"""
   
orig_path = r"data\skyshot-iri\skyshot_iri_hwil_dat.csv"
save_path = r"data\skyshot-iri-z-up\skyshot_iri_hwil_dat.csv"
columns = ["timestamp","temperature","pressure","acc_h_x","acc_h_y","acc_h_z","acc_i_x","acc_i_y","acc_i_z","rot_i_x","rot_i_y","rot_i_z","mag_i_x","mag_i_y","mag_i_z"]
old_up = 2 # 1 2 3 for x y z
new_up = 3 # z up for darkstar
shift = new_up - old_up # will break if old < new
df1 = pd.read_csv(orig_path, header=None, names=columns, index_col=False)
axis_things = ["acc_h_", "acc_i_", "rot_i_", "mag_i_"]
axes = ["x", "y", "z"]
df2 = df1.copy()
for sens in (axis_things):
    for i in range(3):
        col1 = sens + axes[i]
        col2 = sens + axes[(i+shift)%3]
        df2[col2] = df1[col1]
df2.to_csv(save_path, header=False, index=False)
