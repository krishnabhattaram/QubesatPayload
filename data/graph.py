import matplotlib.pyplot as plt
import pandas as pd

import sys

if len(sys.argv) != 2:
    print("Usage: python graph.py <input_file>")
    sys.exit(1)

# Get the input file from the command line argument
input_file = sys.argv[1]

# Load the data
data = pd.read_csv(input_file, sep=" ", names=['Measurement', 'Frequency', 'Status'])

print(data)

# Group by Frequency and Status, then calculate the mean of Measurement
grouped = data.groupby(['Frequency', 'Status'])['Measurement'].mean().unstack()

# Calculate the ratio of 'on' average to 'off' average
grouped['Ratio'] = grouped[1] / grouped[0]

# Plot the data
plt.figure(figsize=(10,5))
plt.plot(grouped['Ratio'])
plt.title('Ratio of Average Measurements for On/Off Status per Frequency')
plt.xlabel('Frequency')
plt.ylabel('Ratio of Average Measurements (On/Off)')
plt.show()

