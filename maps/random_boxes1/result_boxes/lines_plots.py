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

# Threshold the y-axis to 8 milliseconds
y_threshold = 8

# Define coverage ranges
coverage_ranges = [(20, 30), (30, 40), (40, 50), (50, 60), (60, 70)]  # Define your own ranges

# Initialize histograms for each algorithm
hist_rcd = []
hist_rrt_connect = []
hist_bit = []
hist_kpiece = []

# Loop through coverage ranges and calculate mean time for each algorithm
for coverage_range in coverage_ranges:
    # Filter data within coverage range
    rcd_subset = rcd_df[(rcd_df['coverage'] >= coverage_range[0]) & (rcd_df['coverage'] < coverage_range[1])]
    rrt_connect_subset = rrt_connect_df[(rrt_connect_df['coverage'] >= coverage_range[0]) & (rrt_connect_df['coverage'] < coverage_range[1])]
    bit_subset = bit_df[(bit_df['coverage'] >= coverage_range[0]) & (bit_df['coverage'] < coverage_range[1])]
    kpiece_subset = kpiece_df[(kpiece_df['coverage'] >= coverage_range[0]) & (kpiece_df['coverage'] < coverage_range[1])]
    
    # Calculate mean time for each algorithm
    hist_rcd.append(rcd_subset['time_mean'].mean())
    hist_rrt_connect.append(rrt_connect_subset['time_mean'].mean())
    hist_bit.append(bit_subset['time_mean'].mean())
    hist_kpiece.append(kpiece_subset['time_mean'].mean())

# Plot histograms
plt.figure(figsize=(10, 10))


bar_width = 0.2  # Width of each bar
index = range(len(coverage_ranges))  # Index for x-axis

plt.bar(index, hist_rcd, bar_width, label='RCD')
plt.bar([i + bar_width for i in index], hist_rrt_connect, bar_width, label='RRTConnect')
plt.bar([i + 2 * bar_width for i in index], hist_bit, bar_width, label='BIT-RRT')
plt.bar([i + 3 * bar_width for i in index], hist_kpiece, bar_width, label='KPIECE')

plt.xlabel('Map Coverage Range (%)', fontsize=30)  # Increase font size
plt.ylabel('Mean Time (ms)', fontsize=30)  # Increase font size
plt.title('Mean Time per Coverage Range', fontsize=30)  # Increase font size
plt.xticks([i + 1.5 * bar_width for i in index], [f"{range[0]}-{range[1]}" for range in coverage_ranges], fontsize=30)  # Increase font size
plt.yticks(fontsize=30)  # Increase font size
plt.legend(fontsize=30)  # Increase font size
plt.grid(True)
plt.tight_layout()
# plt.savefig('mean_time_histograms.png')
plt.show()
