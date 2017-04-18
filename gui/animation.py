from __future__ import print_function
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import json
import time
import requests

import math

betterformat = []

def handle_close(evt):
    quit()


class Visual(object):
    def __init__(self, ax, period):
        self.ax = ax
        self.point = 0
        self.ms = 0
        self.period = period
        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.xmagoffset = 0
        self.ymagoffset = 0
        self.zmagoffset = -1
        self.time = 0
        self.lastTime = 0
        
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
        
        self.testvel = 0
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
        if 0:
            self.plotStickFigure()

        else:
            self.ms += self.period
            
            self.ax.clear()
            
            #req = requests.get('http://localhost:3000')
            req = requests.get('http://127.0.0.1:3000')
            #req = requests.get('http://192.168.2.50:3000')
            if req.status_code > 299:
                print('Error accessing server: ' + req.status_code)
                quit(-1)
            
            self.lastTime = self.time
                
            data = json.loads(req.text)
            
            if len(data) < 1:
                return
            
            # get multiple readings... TODO
            accelx = (2 * data[len(data) - 1]['accel']['x']) / (2 ** 15)
            accely = (2 * data[len(data) - 1]['accel']['y']) / (2 ** 15)
            accelz = (-(2 * data[len(data) - 1]['accel']['z']) / (2 ** 15)) + 1
            gyrox = (2 * data[len(data) - 1]['gyro']['x']) / (2 ** 15)
            gyroy = (2 * data[len(data) - 1]['gyro']['y']) / (2 ** 15)
            gyroz = (2 * data[len(data) - 1]['gyro']['z']) / (2 ** 15)
            magnetx = data[len(data) - 1]['magno']['x'] * 0.6
            magnety = data[len(data) - 1]['magno']['y'] * 0.6
            magnetz = data[len(data) - 1]['magno']['z'] * 0.6
            self.time = data[len(data) - 1]['timestamp']
            
            if self.lastTime == 0: # first reading
                normalization = math.sqrt((magnetx ** 2) + (magnety ** 2) + (magnetz ** 2))
                #self.xmagoffset = -magnetx/normalization
                #self.ymagoffset = -magnety/normalization
                #self.zmagoffset = -magnetz/normalization
                self.lastTime = self.time
                
            # normalize magnetometer readings
            rsphere = 2
            normalization = math.sqrt((magnetx ** 2) + (magnety ** 2) + (magnetz ** 2))
            magnetx = (magnetx / normalization + self.xmagoffset) * rsphere - rsphere / 2
            magnety = (magnety / normalization + self.ymagoffset) * rsphere - rsphere / 2
            magnetz = (magnetz / normalization + self.zmagoffset) * rsphere - rsphere / 2
            
            #update position
            #self.xPos += accelx * (((self.time - self.lastTime) / 1000) ** 2)
            #self.yPos += accely * (((self.time - self.lastTime) / 1000) ** 2)
            #self.zPos += accelz * (((self.time - self.lastTime) / 1000) ** 2)
            #self.linePlot(self.pelvis, [self.xPos, self.yPos, self.zPos])
            
            self.linePlot(self.pelvis, [magnetx, magnety, magnetz])
#            print(magnetx, "\t", magnety, "\t", magnetz)
            print(magnetx ** 2 + magnety ** 2 + magnetz ** 2)
            
            #print(self.xPos, self.yPos, self.zPos, accelx, accely, accelz, (self.time - self.lastTime)/1000)
            #self.linePlot(self.pelvis, [accelx, accely, accelz])    # plot line for accel data
            #self.linePlot(self.pelvis, [gyrox, gyroy, gyroz])       # plot line for gyro data
            #self.linePlot(self.pelvis, [magnetx, magnety, magnetz]) # plot line for magnetometer data
            #ax.auto_scale_xyz([-25000, 25000], [-25000, 25000], [-25000, 25000])
            bound = rsphere 
            ax.auto_scale_xyz([-bound, bound], [-bound, bound], [-bound, bound])
            #self.ax.clear()
        
        #ax.auto_scale_xyz([-100, 110], [-100, 110], [-100, 110])
        return ax,

if __name__ == "__main__":
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('close_event', handle_close)
    period = 5
    
    v = Visual(ax, period)
    
    # animation updates (ideally) according to the interval in ms (this should be 100 frames per second)
    ani = animation.FuncAnimation(fig, v.update, interval=period)
    
    plt.show()
    
