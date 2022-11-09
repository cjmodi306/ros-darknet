#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = np.array([[4.5, 500, 50],[25.2, 450, 110],[33, 350, 160], [34, 300, 250],[34.2, 200, 260] ])
baseline = data[:,0]
disparity_range = data[:,1]
distance = data[:,2]

plt.rcParams['font.size'] = '28'
	
fig = plt.figure()
ax1 = fig.add_subplot(111)
ax1.plot(baseline, disparity_range, label = 'Disparity Range', color='red')
ax1.set_xlabel('Baseline [in cm]')
ax1.set_ylabel('Disparity Range [in pixels]')
ax1.legend(loc=2)

ax2 = ax1.twinx()
ax2.plot(baseline, distance, label = 'Distance', color='blue')

ax2.set_ylabel('Distance [in cm]')

ax2.legend(loc=1)
ax2.set_title('Verhältnis zwischen Baseline, Disparity Range und Objektabstand', fontweight = 'bold')

for label in (ax1.get_xticklabels() + ax1.get_yticklabels()):
	label.set_fontsize(16)

for label in (ax2.get_xticklabels() + ax2.get_yticklabels()):
	label.set_fontsize(16)

plt.show()

