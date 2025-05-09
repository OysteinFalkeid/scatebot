import pandas as pd
import matplotlib.pyplot as plt

def load_waveform(file_path):
    # Skip metadata and load waveform data
    df = pd.read_csv(file_path, skiprows=26, header=None, usecols=[0, 1])
    df.columns = ['Time', 'Voltage']
    return df.astype(float)

# File paths
file1 = 'DS0001.CSV'
file2 = 'DS0002.CSV'

# Load both waveforms
df1 = load_waveform(file1)
df2 = load_waveform(file2)

# Create first figure
fig1 = plt.figure(figsize=(10, 4))
plt.plot(df1['Time'], df1['Voltage'])
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Waveform 1')
plt.grid(True)

# Create second figure
fig2 = plt.figure(figsize=(10, 4))
plt.plot(df2['Time'], df2['Voltage'], color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Waveform 2')
plt.grid(True)

# Show both figures at once
plt.show()
