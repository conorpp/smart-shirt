import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import json, time, requests, os.path, math

betterformat = []

def handle_close(evt):
    quit()


class SensorManager(object):
    def __init__(self, ):
        
        self.sensors = {'22':[], '23':[], '24':[], '25':[]}
        self.sensor_config = {
                '22':
                {'offsets': {'x':0,
                    'y':0,
                    'z':0}},
                '23':
                {'offsets': {'x':0,
                    'y':0,
                    'z':0}},
                '24':
                {'offsets': {'x':0,
                    'y':0,
                    'z':0}},
                '25':
                {'offsets': {'x':0,
                    'y':0,
                    'z':0}},
                }
        self.aves = {'x':np.zeros(10), 'y':np.zeros(10),'z':np.zeros(10)}

    def clear_sensors(self,):
        self.sensors = {'22':[], '23':[], '24':[], '25':[]}

    def linePlot(self, point1, point2):
        self.ax.plot(
            [np.array(point1[0]), np.array(point2[0])], 
            [np.array(point1[1]), np.array(point2[1])], 
            [np.array(point1[2]), np.array(point2[2])], 
            'k')
            
    def poll_sensor(self,use_offsets=True):
        """
            adds any available data to queue
        """
        req = requests.get('http://127.0.0.1:3000')
        if req.status_code > 299:
            print('Error accessing server: ' + req.status_code)
            quit(-1)
 
        data = json.loads(req.text)

        # organize by sensor ID
        for i in data:
            idx = str(i['id'])

            if use_offsets:
                # apply offsets
                for j in 'xyz':
                    i['magno'][j] -= self.sensor_config[idx]['offsets'][j]
                
                # negate z to make sense
                i['magno']['z'] *= -1

            self.sensors[idx].append(i)

    def calibrate(self,):

        if os.path.isfile('calibration.json'):
            print 'Reusing last calibration settings...'
            print 'Delete \'calibration.json\' to not do this.'
            self.sensor_config = json.loads(open('calibration.json','r').read())
            time.sleep(1)
            return

        seconds = 6

        print "calibration starting in 1 second."
        print
        time.sleep(1)
        print "hold board parallel to ground"
        print
        time.sleep(1)
        print "swing it left to right 180 degrees slowly"
        print
        time.sleep(1)
        print 'collecting samples for %d seconds' % seconds
        print
        time.sleep(0.1)
        for i in range(seconds * 100):
            self.poll_sensor(False)
            time.sleep(1.0/100)
        
        self.set_offsets('xy')
        
        print 'okay now start swinging board up and down 180 degrees slowly'
        print
        print 'collecting samples for %d seconds' % seconds
        print
        time.sleep(0.1)

        for i in range(seconds * 100):
            self.poll_sensor(False)
            time.sleep(1.0/100)

        self.set_offsets('z')
        print
        print 'good job.  all calibrated.  Saving settings to \'calibration.json\''
        print
        open('calibration.json','w+').write(json.dumps(self.sensor_config))
        time.sleep(1)
 
    def set_offsets(self,axis='xyz'):
        for i in ['22','23','24','25']:
            for j in axis:
                jmax = max(self.sensors[i], key=lambda x:x['magno'][j])
                jmin = min(self.sensors[i], key=lambda x:x['magno'][j])

                print '%s (%d, %d)'%( j, jmin['magno'][j] * .6, jmax['magno'][j] * .6)

                offset = (jmax['magno'][j] - jmin['magno'][j])/2.0 + jmin['magno'][j]
                self.sensor_config[i]['offsets'][j] = offset

        self.clear_sensors()

    def _update(self, frame):
        
        """ put data on back of queue """
        self.poll_sensor()

        if not len(self.sensors['22']):
            return

        """ consume 1 data packet per frame from front of queue"""
        pkt = self.sensors['22'].pop(0)

        magnetx = pkt['magno']['x']
        magnety = pkt['magno']['y']
        # invert z axis b/c of board
        magnetz = pkt['magno']['z']

        # this is not needed as it's all relative
        magnetx *= 0.6
        magnety *= 0.6
        magnetz *= 0.6
        
        self.ax.clear()
        self.linePlot([0,0,0], [magnetx, magnety, magnetz])
        
        rsphere = 400
        bound = rsphere 
        self.ax.auto_scale_xyz([-bound, bound], [-bound, bound], [-bound, bound])

        print 'x: %d, y: %d, z: %d' % (magnetx, magnety, magnetz)

    def test(self,):
        period = 10
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', handle_close)
        self.ax = fig.add_subplot(111, projection='3d')
        ani = animation.FuncAnimation(fig, self._update, interval=period)
        plt.show()


if __name__ == "__main__":
    v = SensorManager()
    v.calibrate()
    v.test()
