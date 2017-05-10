import ckbot.logical as L
from time import time as now, sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import collections
c = L.Cluster()
c.populate(4)

command_torque = .75
c.at.NxE4.set_torque(command_torque)
torque = c.at.NxE4.get_torque()

fig = plt.figure()
ax = fig.add_subplot(111)

y2array = collections.deque([None] * 10000, maxlen=10000)
xarray = collections.deque([None] * 10000, maxlen=10000)

l2, = ax.plot(xarray, y2array, 'b-', label = "actual torque")
plt.legend(bbox_to_anchor=(.8, .9), loc=2, borderaxespad=0.)
fig.canvas.draw()
plt.show(block=False)

while True:
	try:
		torque = c.at.NxE4.get_torque()
		
		xarray.append(now())  
		y2array.append(torque)
		if len(xarray) > 100000:
			xarray.popleft()
			y2array.popleft()
		l2.set_xdata(xarray)
		l2.set_ydata(y2array)
		
		ax.relim() 
		ax.autoscale_view(True,True,True)
		fig.canvas.draw()
	except KeyboardInterrupt or ValueError:
		break
c.at.NxE4.set_torque(0)