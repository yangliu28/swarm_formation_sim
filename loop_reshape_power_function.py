# demo of how power function increases the unipolarity of a random distribution
# pass the exponent of the power function in the command line

import matplotlib.pyplot as plt
import numpy as np
import random, time, os, sys

# get the exponent from passing parameter
exponent = float(sys.argv[1])

N = 50  # number of probabilities in the distribution

x_data = np.array(range(N))
y_data = np.array([random.random() for i in range(N)])
y_data_sum = np.sum(y_data)
y_data = y_data/y_data_sum

fig = plt.figure()
ax = fig.add_subplot(111)
rects = ax.bar(x_data, y_data, align='center')
ax.set_xlim(-1, N)
ax.set_ylim(0.0, 1.0)
fig.canvas.draw()
fig.show()

while True:
    sys.stdout.write('+')
    sys.stdout.flush()

    y_data = np.power(y_data, exponent)  # the power function
    y_data_sum = np.sum(y_data)
    y_data = y_data/y_data_sum

    for i in range(N):
        rects[i].set_height(y_data[i])
    fig.canvas.draw()
    fig.show()

    time.sleep(0.3)

