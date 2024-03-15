import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV files
rcd_df = pd.read_csv("rcd.csv")
kpiece_df = pd.read_csv("KPIECE1.csv")
bit_df =  pd.read_csv("BIT.csv")
# pdst_df =  pd.read_csv("PDST.csv")
rrt_connect_df =  pd.read_csv("RRTConnect.csv")

# Convert time mean and std from seconds to milliseconds
rcd_df['time_mean'] *= 1000
rrt_connect_df['time_mean'] *= 1000
bit_df['time_mean'] *= 1000
# pdst_df['time_mean'] *= 1000
kpiece_df['time_mean'] *= 1000

rcd_df['time_std'] *= 1000
rrt_connect_df['time_std'] *= 1000
bit_df['time_std'] *= 1000
# pdst_df['time_std'] *= 1000
kpiece_df['time_std'] *= 1000

# Convert length mean and std from seconds to milliseconds
rcd_df['length_mean'] *= 1000
rrt_connect_df['length_mean'] *= 1000
bit_df['length_mean'] *= 1000
# pdst_df['time_mean'] *= 1000
kpiece_df['length_mean'] *= 1000

rcd_df['length_std'] *= 1000
rrt_connect_df['length_std'] *= 1000
bit_df['length_std'] *= 1000
# pdst_df['time_std'] *= 1000
kpiece_df['length_std'] *= 1000

# Threshold the y-axis to 8 milliseconds
y_threshold = 8

# Plot for time
plt.figure(figsize=(10, 5))
plt.errorbar(rcd_df.index + 1, rcd_df['time_mean'], yerr=rcd_df['time_std'], label='RCD', fmt='o', capsize=5)
plt.errorbar(rrt_connect_df.index + 1, rrt_connect_df['time_mean'], yerr=rrt_connect_df['time_std'], label='RRTConnect', fmt='o', capsize=5)
plt.errorbar(bit_df.index + 1, bit_df['time_mean'], yerr=bit_df['time_std'], label='BIT', fmt='o', capsize=5)
# plt.errorbar(pdst_df.index + 1, pdst_df['time_mean'], yerr=pdst_df['time_std'], label='PDST', fmt='o', capsize=5)
plt.errorbar(kpiece_df.index + 1, kpiece_df['time_mean'], yerr=kpiece_df['time_std'], label='KPIECE', fmt='o', capsize=5)

plt.xlabel('Experiment Number')
plt.ylabel('Mean time (ms)')
plt.title('Mean time per maze experiment')
plt.legend()
plt.grid(True)
plt.ylim(0, y_threshold)  # Set y-axis limit to 8 milliseconds
plt.tight_layout()
plt.savefig('time_mean_with_error_bars.png')
plt.show()

# Plot for length
plt.figure(figsize=(10, 5))
plt.errorbar(rcd_df.index + 1, rcd_df['length_mean'], yerr=rcd_df['length_std'], label='RCD', fmt='o', capsize=5)
plt.errorbar(rrt_connect_df.index + 1, rrt_connect_df['length_mean'], yerr=rrt_connect_df['length_std'], label='RRTConnect', fmt='o', capsize=5)
plt.errorbar(bit_df.index + 1, bit_df['length_mean'], yerr=bit_df['length_std'], label='BIT', fmt='o', capsize=5)
# plt.errorbar(pdst_df.index + 1, pdst_df['time_mean'], yerr=pdst_df['time_std'], label='PDST', fmt='o', capsize=5)
plt.errorbar(kpiece_df.index + 1, kpiece_df['length_mean'], yerr=kpiece_df['length_std'], label='KPIECE', fmt='o', capsize=5)

plt.xlabel('Experiment Number')
plt.ylabel('Mean path length (ms)')
plt.title('Mean path length per maze experiment')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('length_mean_with_error_bars.png')
plt.show()
