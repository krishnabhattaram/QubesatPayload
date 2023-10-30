import matplotlib.pyplot as plt
import pandas as pd

import sys

if len(sys.argv) != 2:
    print("Usage: python graph2.py <input_file>")
    sys.exit(1)

# Get the input file from the command line argument
input_file = sys.argv[1]

# Read the data from screenlog.1 using pandas
data = pd.read_csv(input_file, header=None, names=['Y'])

# Generate the x-axis values
x_values = list(range(2800, 2951))

# Plot the average of the data across index 0, 151, 2*151, etc.
averaged_data = []
for i in range(151):
    indices = range(i, len(data), 151)
    average = data['Y'][indices].mean()
    averaged_data.append(average)

plt.plot(x_values, averaged_data)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Data Plot')
plt.show()
