# test for dynamic bar chart

import matplotlib.pyplot as plt
import numpy, random, time


# x_pos = numpy.arrange(5)
x_pos = range(1,6)


while True:
    y_pos = [random.random() for i in range(5)]
    plt.bar(x_pos, y_pos, align='center')
    plt.show()
    time.sleep(0.5)


