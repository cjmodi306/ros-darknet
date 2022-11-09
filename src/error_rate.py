#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = np.array([[300, 10.7],[350,8],[400,7],[450,7.8],[500,8],[550,8.8],[600,10.7],[650,11.4]])
disparity_range = data[:,0]
error_rate = data[:,1]

fig = plt.figure()
ax1 = fig.add_subplot(111)
ax1.plot(disparity_range, error_rate, label = 'Disparity Range', color='red')
ax1.set_xlabel('Baseline')
ax1.set_ylabel('Disparity Range')
ax1.legend(loc=2)

plt.show()
