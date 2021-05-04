import matplotlib.pyplot as plt
import csv

current = []
soc = []
procentSOC = []
batteryState = []
batteryMode = []

with open('/home/kamil/catkin_ws/src/soc/src/current.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        current.append(row[1])
    current = [float(i) for i in current[1:]]

with open('/home/kamil/catkin_ws/src/soc/src/current.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        soc.append(row[0])
    soc = [float(i) for i in soc[1:]]

with open('/home/kamil/catkin_ws/src/soc/src/feedback.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        procentSOC.append(row[2])
    procentSOC = [float(i) for i in procentSOC[1:]]

with open('/home/kamil/catkin_ws/src/soc/src/feedback.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        batteryState.append(row[0])
    batteryState = [float(i) for i in batteryState[1:]]

with open('/home/kamil/catkin_ws/src/soc/src/feedback.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        batteryMode.append(row[1])
    batteryMode = [float(i) for i in batteryMode[1:]]

# fig, (ax1, ax2, ax3) = plt.subplots(3)

# fig.suptitle('Random state')

# ax1.plot(current)
# ax1.set(xlabel='Sample', ylabel='Current [A]')
# ax1.grid()

# ax2.plot(soc)
# ax2.set(xlabel='Sample', ylabel='SOC [As]')
# ax2.grid()

# ax3.plot(procentSOC)
# ax3.set(xlabel='Sample', ylabel='SOC [%]')
# ax3.grid()

# plt.savefig('/home/kamil/charging.png')

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4)

fig.suptitle('Random state')

ax1.plot(current)
ax1.set(xlabel='Sample', ylabel='Current [A]')
ax1.grid()

ax2.plot(procentSOC)
ax2.set(xlabel='Sample', ylabel='SOC [%]')
ax2.grid()

ax3.plot(batteryState)
ax3.set(xlabel='Sample', ylabel='Battery State')
ax3.grid()

ax4.plot(batteryState)
ax4.set(xlabel='Sample', ylabel='Battery Mode')
ax4.grid()

plt.savefig('/home/kamil/random.png')