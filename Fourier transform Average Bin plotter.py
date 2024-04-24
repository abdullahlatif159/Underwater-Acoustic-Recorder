import matplotlib.pyplot as plt
import numpy as np

# Assuming the data format in 'test at 22k.txt' has been corrected
plt.rcParams.update({'font.size': 35, 'axes.labelsize': 35, 'axes.titlesize': 35, 'xtick.labelsize': 35, 'ytick.labelsize': 35})
plt.rc('font', family='serif', serif='calibri')
# Your plotting code here

# Load the data
data_path = 'C:/Users/abdul/Downloads/test at 22k.txt'  # Update this to the correct path of your file
frequencies = []
intensities = []

# Read the file line by line
with open(data_path, 'r') as file:
    for line in file:
        try:
            # Parse the frequency and intensity from each line
            frequency, intensity = line.strip().split(',')
            frequency = float(frequency.split(':')[1].strip().split(' ')[0])
            intensity = float(intensity.split(':')[1].strip())
            frequencies.append(frequency)
            intensities.append(intensity)
        except ValueError:
            # Ignore lines with formatting issues
            continue

# Convert lists to numpy arrays for easier manipulation
frequencies = np.array(frequencies)
intensities = np.array(intensities)

# Define the number of bins
num_bins = 500  # Or any other number you prefer

# Create bins and calculate the average intensity in each bin
bin_edges = np.linspace(frequencies.min(), frequencies.max(), num_bins+1)
bin_indices = np.digitize(frequencies, bin_edges, right=True)
average_intensities = np.array([intensities[bin_indices == i].mean() for i in range(1, len(bin_edges))])

# Prepare data for plotting
# Use the middle point of each bin as the x-axis value
bin_middles = (bin_edges[:-1] + bin_edges[1:]) / 2

# Plot the data
plt.figure(figsize=(14, 7))
plt.bar(bin_middles, average_intensities, width=np.diff(bin_edges), edgecolor='black')
plt.title('Fourier transform analysis of audio wave form')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Average Intensity')
plt.grid(True)
plt.yscale('log')  # Use logarithmic scale if intensity varies widely
plt.show()
