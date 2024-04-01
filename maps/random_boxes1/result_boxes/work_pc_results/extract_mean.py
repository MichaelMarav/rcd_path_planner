import pandas as pd
import numpy as np

def split_data(data, threshold_ranges):
    stacks = [[] for _ in range(len(threshold_ranges) + 1)]
    for i in range(data.shape[0]):
        coverage = data[i, 0]
        value = data[i, 3]
        for idx, (lower, upper) in enumerate(threshold_ranges):
            if lower <= coverage < upper:
                stacks[idx].append(value)
                break
        else:
            stacks[-1].append(value)  # For coverage out of any range
    return stacks

def calculate_mean_std(stacks):
    means = [np.mean(stack) for stack in stacks]
    stds = [np.std(stack) for stack in stacks]
    return means, stds

def print_means_stds(algorithm, means, stds, coverage_ranges):
    print(f"Algorithm: {algorithm}")
    for i, (lower, upper) in enumerate(coverage_ranges):
        print(f"Mean and Standard Deviation for Stack {lower}-{upper}:")
        print(f"Mean: {means[i]}, Standard Deviation: {stds[i]}")
    print()

# Read the CSV files
rcd_df = pd.read_csv("rcd.csv")
kpiece_df = pd.read_csv("KPIECE1.csv")
bit_df = pd.read_csv("BIT.csv")
rrt_connect_df = pd.read_csv("RRTConnect.csv")

# Remove the first row for each DataFrame
rcd = rcd_df.iloc[1:].values
kpiece = kpiece_df.iloc[1:].values
bit = bit_df.iloc[1:].values
rrt_connect = rrt_connect_df.iloc[1:].values

# Define coverage ranges
coverage_ranges = [(20, 30), (30, 40), (40, 50), (50, 60), (60, 70)]

# Split data and calculate mean and std for each algorithm
rcd_stacks = split_data(rcd, coverage_ranges)
kpiece_stacks = split_data(kpiece, coverage_ranges)
bit_stacks = split_data(bit, coverage_ranges)
rrt_connect_stacks = split_data(rrt_connect, coverage_ranges)

rcd_means, rcd_stds = calculate_mean_std(rcd_stacks)
kpiece_means, kpiece_stds = calculate_mean_std(kpiece_stacks)
bit_means, bit_stds = calculate_mean_std(bit_stacks)
rrt_connect_means, rrt_connect_stds = calculate_mean_std(rrt_connect_stacks)

# Print means and stds for each algorithm
print_means_stds("RCD", rcd_means, rcd_stds, coverage_ranges)
print_means_stds("KPIECE", kpiece_means, kpiece_stds, coverage_ranges)
print_means_stds("BIT", bit_means, bit_stds, coverage_ranges)
print_means_stds("RRTConnect", rrt_connect_means, rrt_connect_stds, coverage_ranges)
