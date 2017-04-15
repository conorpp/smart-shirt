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


    def draw(self,):
        glBegin(GL_LINES)
        for edge in self.edges:
            for vertex in edge:
                glVertex3fv(self.points[vertex])
        glEnd()


def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(0.0,0.0, -8)
    cube = Cube()
    i = 0
    cube.move([.1,.1,.1])
    #cube.rotate([0,0,10], 2 * 3.14/8);

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()


        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # use move and rotate methods.
        # add a cube for each sensor being used.
        # can change cube to something else if needed.
        cube.move([0,float(i-100)/4000,0])
        cube.rotate([0,0,10], 0.009);
        i = (i + 1) % 200


        cube.draw()
        pygame.display.flip()
        pygame.time.wait(10)


main()
