import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV files
rcd_df = pd.read_csv("rcd.csv")
kpiece_df = pd.read_csv("KPIECE1.csv")
bit_df = pd.read_csv("BIT.csv")
# pdst_df = pd.read_csv("PDST.csv")
rrt_connect_df = pd.read_csv("RRTConnect.csv")

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

# Sort the RCD dataframe by mean values in descending order
rcd_sorted = rcd_df.sort_values(by='time_mean', ascending=False)

# Ignore the first 10 largest elements and consider the next 90 samples
rcd_sorted = rcd_sorted.iloc[10:100]

# Plot for time
plt.figure(figsize=(10, 10))

# Plot RCD
plt.plot(range(len(rcd_sorted)), rcd_sorted['time_mean'], label='RCD', marker='o', alpha=0.5)

# Plot other datasets without changing their order
for df, label in [(rrt_connect_df, 'RRTConnect'), (bit_df, 'BIT'), (kpiece_df, 'KPIECE1')]:
    plt.plot(range(len(rcd_sorted)), df.loc[rcd_sorted.index, 'time_mean'], label=label, marker='o', alpha=0.5)

plt.xlabel('Experiment Number', fontsize=25)
plt.ylabel('Mean time (ms)', fontsize=25)
plt.title('Mean time for maze experiments',fontsize=30)
plt.legend(fontsize=30)
plt.grid(True)
plt.ylim(0, y_threshold)  # Set y-axis limit to 8 milliseconds
plt.tight_layout()
plt.yticks(fontsize=25)  # Increase font size
plt.xticks(fontsize=25)
plt.savefig('time_mean.png')
plt.show()

# Plot for length
plt.figure(figsize=(10, 10))

# Plot RCD
plt.plot(range(len(rcd_sorted)), rcd_sorted['length_mean'], label='RCD', marker='o', alpha=0.5)

# Plot other datasets without changing their order
for df, label in [(rrt_connect_df, 'RRTConnect'), (bit_df, 'BIT'), (kpiece_df, 'KPIECE1')]:
    plt.plot(range(len(rcd_sorted)), df.loc[rcd_sorted.index, 'length_mean'], label=label, marker='o', alpha=0.5)

plt.xlabel('Experiment Number',fontsize=25)
plt.ylabel('Length Mean (ms)',fontsize=25)
plt.title('Length Mean',fontsize=30)
plt.legend(fontsize=30)
plt.grid(True)
plt.tight_layout()

plt.yticks(fontsize=25)  # Increase font size
plt.xticks(fontsize=25)
plt.savefig('length_mean.png')
plt.show()
