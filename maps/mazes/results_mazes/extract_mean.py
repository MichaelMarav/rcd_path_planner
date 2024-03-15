import pandas as pd

# Read the CSV files
rcd_df = pd.read_csv("rcd.csv")
kpiece_df = pd.read_csv("KPIECE1.csv")
bit_df = pd.read_csv("BIT.csv")
rrt_connect_df = pd.read_csv("RRTConnect.csv")

# Calculate mean of 'length_mean' for each file
rcd_mean = rcd_df['length_mean'].mean()
kpiece_mean = kpiece_df['length_mean'].mean()
bit_mean = bit_df['length_mean'].mean()
rrt_connect_mean = rrt_connect_df['length_mean'].mean()

# Print the mean for each file
print("Mean of length_mean in rcd.csv:", rcd_mean)
print("Mean of length_mean in KPIECE1.csv:", kpiece_mean)
print("Mean of length_mean in BIT.csv:", bit_mean)
print("Mean of length_mean in RRTConnect.csv:", rrt_connect_mean)