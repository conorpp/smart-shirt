import pygame
import numpy as np
import math
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *



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
    def __init__(self,):
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
        self.origin = np.array([0,0,0])
        self.offset = np.array([0,0,0])

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
    def scale(self, s):
        self.points *= s

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
        self.hand = Cube()
        self.wrist    = Cube()
        self.elbow    = Cube()
        self.uparm    = Cube()
        self.shoulder = Cube()

        #self.shoulder.scale(.6)
        #self.elbow.scale(.6)
        #self.hand.scale(.6)

        # 10 in == 254 mm
        self.a = 0.254

        self.shoulder.move([0,self.a,0])
        self.uparm.move([0,self.a/2,0])
        # elbow in center

        self.wrist.move([0,-self.a/2,0])
        self.hand.move([0,-self.a,0])

    def draw(self,):
        self.hand.draw()
        self.wrist.draw()
        self.uparm.draw()
        self.elbow.draw()
        self.shoulder.draw()
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
        self.uparm.move_to(self.shoulder.origin)
        self.uparm.move(np.array(v) * (-self.a/2))
        self.set_elbow(v)

    def set_elbow(self,v):
        self.elbow.move_to(self.shoulder.origin)
        self.elbow.move(np.array(v) * (-self.a))

    def set_wrist(self,v):
        self.wrist.move_to(self.elbow.origin)
        self.wrist.move(np.array(v) * (-self.a/2))
        self.set_hand(v)

    def set_hand(self,v):
        self.hand.move_to(self.elbow.origin)
        self.hand.move(np.array(v) * (-self.a))



def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(0.0,0.0, -1)
    i = 0
    arm = Arm()


    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()


        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # use move and rotate methods.
        # add a cube for each sensor being used.
        # can change cube to something else if needed.
        #cube.move([0,0,float(i-100)/4000])
        #cube.rotate([0,0,10], 0.009);
        arm.set_uparm([math.cos(i),math.sin(i),math.cos(i)])
        arm.set_wrist([-math.cos(i),math.sin(i),math.sin(i)])
        arm.draw()
        i = (i + .01)


        pygame.display.flip()
        pygame.time.wait(10)


main()
