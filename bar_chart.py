# test for dynamic bar chart

import matplotlib.pyplot as plt
import numpy, random, time

fig = plt.figure()
ax = fig.add_subplot(111)


x_pos = numpy.arange(5)


while True:
    # plt.clf()
    # y_pos = [random.random() for i in range(5)]
    # plt.bar(x_pos, y_pos, align='center')
    # plt.show()
    y_pos = [random.random() for i in range(5)]
    ax.clear()
    ax.bar(x_pos, y_pos, align='center')
    fig.canvas.draw()
    fig.show()
    time.sleep(0.5)




