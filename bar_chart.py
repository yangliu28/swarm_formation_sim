# test for bar chart animation

import matplotlib.pyplot as plt
from matplotlib import gridspec
import numpy, random, time

fig = plt.figure(figsize=(8,4), tight_layout=True)
gs = gridspec.GridSpec(1, 2, width_ratios=[1,1])
ax1 = fig.add_subplot(gs[0])
ax2 = fig.add_subplot(gs[1])

x_pos = numpy.arange(5)

while True:
    ax1.clear()
    ax2.clear()
    y1_pos = [random.random() for i in range(5)]
    y2_pos = [random.random() for i in range(5)]
    ax1.bar(x_pos, y1_pos, align='center')
    ax2.bar(x_pos, y2_pos, align='center')
    fig.canvas.draw()
    fig.show()
    time.sleep(0.5)



