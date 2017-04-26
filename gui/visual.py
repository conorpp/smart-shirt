import pygame
import numpy as np
import numpy.linalg as la
import math
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

from sensors import SensorManager



def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]]) 

class Cube():
    def __init__(self,start_pos=[0,0,0]):
        self.points = np.array([
                [1., -1., -1.],
                [1., 1., -1.],
                [-1., 1., -1.],
                [-1., -1., -1.],
                [1., -1., 1.],
                [1., 1., 1.],
                [-1., -1., 1.],
                [-1., 1., 1.]
                ])
        self.points = self.points /50
        self.points_copy = np.copy(self.points)
        self.origin = np.array([0,0,0])


        self.edges = (
                (0,1),
                (0,3),
                (0,4),
                (2,1),
                (2,3),
                (2,7),
                (6,3),
                (6,4),
                (6,7),
                (5,1),
                (5,4),
                (5,7)
                )
        self.start = np.array(start_pos)
        self.move(self.start)

    def scale(self, s):
        self.points *= s

    def reset(self,):
        self.points = np.copy(self.points_copy)
        self.origin = np.array([0,0,0])

    def rotate(self,axis, rads):

        old_org = np.array(self.origin)
        self.move(-self.origin)

        axis = np.array(axis)

        for i,val in enumerate(self.points):
            self.points[i] = np.dot(rotation_matrix(axis,rads), val)

        self.move(old_org)

    def move(self,v):
        v = np.array(v)
        self.origin = self.origin + v
        for i,val in enumerate(self.points):
            self.points[i] = val + v

    def move_to(self,v):
        self.move(-self.origin)
        self.move(v)


    def draw(self,):
        glBegin(GL_LINES)
        for edge in self.edges:
            for vertex in edge:
                glVertex3fv(self.points[vertex])
        glEnd()

class Arm():
    def __init__(self,):

        # 10 in == 254 mm
        self.a = 0.254

        self.hand = Cube([0,-self.a,0])
        self.wrist    = Cube([0,-self.a/2,0])
        self.elbow    = Cube([0,self.a/20,0])
        self.uparm    = Cube([0,self.a/2,0])
        self.shoulder = Cube([0,self.a,0])

    def angle3d(self, v1, v2):
        """ calculate angle radians between 2 normalized 3d vectors """
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

    def rotate_part(self, part, v):
        """ rotate a part so it still aligns with it's starting position for a vector it moved about """
        n = part.start / abs(part.start).max()
        v = np.array(v)
        angle = self.angle3d(v, n)
        axis = -np.cross(v, n)
        part.rotate(axis, angle)


    def draw(self,):
        glColor3f(0.9,0.5,0.2) # orange
        self.hand.draw()
        glColor3f(0.2,0.5,0.9) # blue
        self.wrist.draw()
        glColor3f(0.2,0.5,0.9) # blue
        self.uparm.draw()
        glColor3f(0.9,0.5,0.2) # orange
        self.elbow.draw()
        glColor3f(0.9,0.5,0.2) # orange
        self.shoulder.draw()

        glColor3f(1,1,1) # white
        glBegin(GL_LINES)

        glVertex3fv(self.hand.origin)
        glVertex3fv(self.wrist.origin)

        glVertex3fv(self.wrist.origin)
        glVertex3fv(self.elbow.origin)

        glVertex3fv(self.elbow.origin)
        glVertex3fv(self.uparm.origin)

        glVertex3fv(self.uparm.origin)
        glVertex3fv(self.shoulder.origin)
        glEnd()

    def set_uparm(self,v):
        
        self.uparm.reset()
        self.uparm.move(self.shoulder.origin)
        self.uparm.move(np.array(v) * (-self.a/2))
        self.rotate_part(self.uparm, v)

        self.set_elbow(v)

    def set_elbow(self,v):
        self.elbow.reset()
        self.elbow.move(self.shoulder.origin)
        self.elbow.move(np.array(v) * (-self.a))
        self.rotate_part(self.elbow, v)

    def set_wrist(self,v):
        self.wrist.reset()
        self.wrist.move(self.elbow.origin)
        self.wrist.move(np.array(v) * (-self.a/2))
        
        self.rotate_part(self.wrist, v)
        
        self.set_hand(v)

    def set_hand(self,v):
        self.hand.reset()
        self.hand.move(self.elbow.origin)
        self.hand.move(np.array(v) * (-self.a))
        self.rotate_part(self.hand, v)



def visualize(sens):
    pygame.init()
    display = (1200,1000)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(-0.25,0.25, -2)
    i = 0
    arm = Arm()
    pygame.display.flip()

    sens.clear_sensors()


    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        if not sens.has_data('22') or not sens.has_data('23'):
            sens.poll_sensor()

        xyz = sens.get_point('22')
        if None not in xyz:
            xyz = sens.lowpass('22', xyz, 15)
            xyz = sens.get_point_normal(xyz)
            arm.set_uparm(xyz)

        xyz = sens.get_point('23')
        if None not in xyz:
            xyz = sens.lowpass('23', xyz, 15)
            xyz = sens.get_point_normal(xyz)
            arm.set_wrist(xyz)
        
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)


        arm.draw()

        # updates the display
        pygame.display.flip()
        pygame.time.wait(2)


if __name__ == "__main__":
    s = SensorManager()
    s.calibrate()
    s.poll_sensor()
    s.clear_sensors()

    visualize(s)

