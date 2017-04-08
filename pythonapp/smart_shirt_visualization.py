from __future__ import print_function
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import json
import time

def handle_close(evt):
	quit()


class Visual(object):
	def __init__(self, ax, data, period):
		self.ax = ax
		self.data = data
		self.point = 0
		self.ms = 0
		self.period = 0
		#totally guessing that this is the factor required to convert accel data to cm/s^2
		self.accelScale = 0.68
		
		#Body points, line verticies [x, y, z] measurements based on self
		# scale in cm (ideally)
		# may have to add velocity?
		self.leftWrist = [0, -20, -15]
		self.leftElbow  = [0, -20, 20]
		self.leftShoulder  = [0, -20, 50]
		self.leftKnee  = [0, -10, -38]
		self.leftAnkle  = [0, -10, -80]
		self.pelvis  = [0, 0, 0]
		self.rightWrist = [0, 20, -15]
		self.rightElbow  = [0, 20, 20]
		self.rightShoulder = [0, 20, 50]
		self.rightKnee  = [0, 10, -38]
		self.rightAnkle  = [0, 10, -80]
		
		self.testvel = [0, 0, 0]
		self.testpoint = [0, 0, 0]
		ax.auto_scale_xyz([-100, 110], [-100, 110], [-100, 110])

	def linePlot(self, point1, point2):
		self.ax.plot(
			[np.array(point1[0]), np.array(point2[0])], 
			[np.array(point1[1]), np.array(point2[1])], 
			[np.array(point1[2]), np.array(point2[2])], 
			'k')
			
	def plotStickFigure(self):
		self.linePlot(self.leftWrist, self.leftElbow)
		self.linePlot(self.leftElbow, self.leftShoulder)
		self.linePlot(self.leftShoulder, self.pelvis)
		self.linePlot(self.pelvis, self.leftKnee)
		self.linePlot(self.leftKnee, self.leftAnkle)
		self.linePlot(self.rightWrist, self.rightElbow)
		self.linePlot(self.rightElbow, self.rightShoulder)
		self.linePlot(self.pelvis, self.rightShoulder)
		self.linePlot(self.rightShoulder, self.leftShoulder)
		self.linePlot(self.pelvis, self.rightKnee)
		self.linePlot(self.rightKnee, self.rightAnkle)
		
	def update(self, frame):
		self.ms += 1
		if self.ms > self.data[self.point]['timestamp']:
			self.ms = 0
			self.point = self.point + 1 % len(data)
		
		self.ax.clear()
		self.testpoint[0] = self.data[self.point]['accel']['x']
		self.testpoint[1] = self.data[self.point]['accel']['y']
		self.testpoint[2] = self.data[self.point]['accel']['z']
		
		#Compute new velocities
		#self.testvel[0] = (xaccel / 1000) + self.testvel[0]
		#self.testvel[1] = (yaccel / 1000) + self.testvel[1]
		#self.testvel[2] = (zaccel / 1000) + self.testvel[2]
		#compute new test point:
		#self.testpoint[0] = (self.testvel[0] / 1000) + self.testpoint[0]
		#self.testpoint[1] = (self.testvel[1] / 1000) + self.testpoint[1]
		#self.testpoint[2] = (self.testvel[2] / 1000) + self.testpoint[2]
		point = [self.testpoint[0] - 1, self.testpoint[1], self.testpoint[2]]
		self.linePlot(point, self.testpoint)
		scaler = 25000
		ax.auto_scale_xyz([-scaler, scaler], [-scaler, scaler], [-scaler, scaler])
		#self.ax.clear()
		#self.plotStickFigure()
		#ax.auto_scale_xyz([-100, 110], [-100, 110], [-100, 110])
		return ax,

if __name__ == "__main__":
	
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	fig.canvas.mpl_connect('close_event', handle_close)
	
	data = json.loads(open('testvector_circles.json.txt', 'r').read())

	#Convert timestamps to the differences between them
	period = 0
	x = []
	xave = 0
	y = []
	yave = 0
	z = []
	zave = 0
	for i in range(len(data)):
		if i + 1 == len(data):
			data[i]['timestamp'] = 12 # 12 was the average time
		else:
			data[i]['timestamp'] = data[i + 1]['timestamp'] - data[i]['timestamp']
		period += data[i]['timestamp']
		
	v = Visual(ax, data, period)
	
	# animation updates (ideally) according to the interval in ms (this should be 100 frames per second)
	ani = animation.FuncAnimation(fig, v.update, interval=1)
	
	plt.show()
