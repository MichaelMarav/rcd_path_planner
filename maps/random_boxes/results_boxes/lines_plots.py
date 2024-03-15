import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV files
rcd_df = pd.read_csv("rcd.csv")
kpiece_df = pd.read_csv("KPIECE1.csv")
bit_df =  pd.read_csv("BIT.csv")
# pdst_df =  pd.read_csv("PDST.csv")
rrt_connect_df =  pd.read_csv("RRTConnect.csv")

# Convert time mean from seconds to milliseconds
rcd_df['time_mean'] *= 1000
rrt_connect_df['time_mean'] *= 1000
bit_df['time_mean'] *= 1000
# pdst_df['time_mean'] *= 1000
kpiece_df['time_mean'] *= 1000

# Convert length mean from seconds to milliseconds
# rcd_df['length_mean'] *= 1000
# rrt_connect_df['length_mean'] *= 1000
# bit_df['length_mean'] *= 1000
# # pdst_df['length_mean'] *= 1000
# kpiece_df['length_mean'] *= 1000

# Threshold the y-axis to 8 milliseconds
y_threshold = 8

# Sort the dataframes by mean values in descending order only for RCD
rcd_df = rcd_df.sort_values(by='time_mean', ascending=False)

# Print experiment numbers of RCD in descending order
print("Experiment numbers of RCD in descending order:", rcd_df.index.tolist())

# Plot for time
plt.figure(figsize=(10, 5))

# Plot RCD
plt.plot(range(len(rcd_df)), rcd_df['time_mean'], label='RCD', marker='o', alpha=0.5)

# Plot other datasets without changing their order
plt.plot(range(len(rrt_connect_df)), rrt_connect_df['time_mean'], label='RRTConnect', marker='o', alpha=0.5)
plt.plot(range(len(bit_df)), bit_df['time_mean'], label='BIT', marker='o', alpha=0.5)
# plt.plot(range(len(pdst_df)), pdst_df['time_mean'], label='PDST', marker='o', alpha=0.5)
plt.plot(range(len(kpiece_df)), kpiece_df['time_mean'], label='KPIECE', marker='o', alpha=0.5)

plt.xlabel('Experiment Number')
plt.ylabel('Time Mean (ms)')
plt.title('Time Mean')
plt.legend()
plt.grid(True)
plt.ylim(0, y_threshold)  # Set y-axis limit to 8 milliseconds
plt.tight_layout()
plt.savefig('time_mean.png')
plt.show()

# Plot for length
plt.figure(figsize=(10, 5))

# Plot RCD
plt.plot(range(len(rcd_df)), rcd_df['length_mean'], label='RCD', marker='o', alpha=0.5)

# Plot other datasets without changing their order
plt.plot(range(len(rrt_connect_df)), rrt_connect_df['length_mean'], label='RRTConnect', marker='o', alpha=0.5)
plt.plot(range(len(bit_df)), bit_df['length_mean'], label='BIT', marker='o', alpha=0.5)
# plt.plot(range(len(pdst_df)), pdst_df['length_mean'], label='PDST', marker='o', alpha=0.5)
plt.plot(range(len(kpiece_df)), kpiece_df['length_mean'], label='KPIECE', marker='o', alpha=0.5)

plt.xlabel('Experiment Number')
plt.ylabel('Length Mean (ms)')
plt.title('Length Mean')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('length_mean.png')
plt.show()
