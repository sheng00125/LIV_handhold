import matplotlib.pyplot as plt
import tkinter
with open('inc_time', 'r') as f:
    lines = f.readlines()
    x = [float(line.split()[0]) for line in lines]
    # y = [float(line.split()[1]) for line in lines]
plt.plot(range(0, len(lines)), x, 1)
plt.show()