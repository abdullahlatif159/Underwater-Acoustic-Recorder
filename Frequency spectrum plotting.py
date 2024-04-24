import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

# Load the data
data = pd.read_csv("C:/Users/abdul/Desktop/audacity spectrum.txt", sep='\t')

# Define the frequency markers
freq_markers = [2,3,4,6,10,14,20,30, 50, 100, 200, 400,610, 1000, 2000, 4000, 10000, 20000, 40000, 80000]

# Plotting the frequency spectrum with a filled area under the line, with dark purple color
plt.figure(figsize=(14, 7))
plt.fill_between(data['Frequency (Hz)'], data['Level (dB)'], -72, color="#8c3cbe", alpha=1)
plt.plot(data['Frequency (Hz)'], data['Level (dB)'], color="Slateblue", alpha=0.6, linewidth=0)

plt.title('Frequency Spectrum of Audio Waveform')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Level (dB)')
plt.xscale('log')  # Set the x-axis to a logarithmic scale

# Set custom x-axis tick positions and labels
plt.xticks(freq_markers, [f'{x}Hz' if x < 1000 else f'{int(x/1000)}kHz' for x in freq_markers])

plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.ylim(-72, data['Level (dB)'].max() + 10)
plt.show()
