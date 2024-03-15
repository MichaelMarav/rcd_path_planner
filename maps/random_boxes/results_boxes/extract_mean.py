import pandas as pd

# Read the CSV files
rcd_df = pd.read_csv("rcd.csv")
kpiece_df = pd.read_csv("KPIECE1.csv")
bit_df = pd.read_csv("BIT.csv")
rrt_connect_df = pd.read_csv("RRTConnect.csv")

# Create bins for coverage ranges
bins = [20, 30, 40, 50, 60, 70]

# Function to calculate mean length for each coverage batch
def calculate_mean_length(df):
    df['coverage_bin'] = pd.cut(df['coverage'], bins=bins)
    result1 = df.groupby('coverage_bin')['length_mean'].mean()
    result2 = df.groupby('coverage_bin')['length_mean'].std()

    return result1, result2

  
# Calculate mean length for each algorithm
rcd_mean_length, rcd_std = calculate_mean_length(rcd_df)
kpiece_mean_length,kpiece_std = calculate_mean_length(kpiece_df)
bit_mean_length,bit_std = calculate_mean_length(bit_df)
rrt_connect_mean_length, rrtconnect_std = calculate_mean_length(rrt_connect_df)

# Print the results
print("Mean length for RCD:")
print(rcd_mean_length , "  " , rcd_std)
print("\nMean length for KPIECE:")
print(kpiece_mean_length, "  ", kpiece_std)
print("\nMean length for BIT:")
print(bit_mean_length, " ", bit_std)
print("\nMean length for RRTConnect:")
print(rrt_connect_mean_length, "  " ,rrtconnect_std)
