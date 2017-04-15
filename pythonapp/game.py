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
                [1, -1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, -1, -1],
                [1, -1, 1],
                [1, 1, 1],
                [-1, -1, 1],
                [-1, 1, 1]
                ])
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
    def rotate(self,):
        pass
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

    glTranslatef(0.0,0.0, -5)
    cube = Cube()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        #glTranslatef(0.00, 0.00, 0.00)
        #glRotatef(-10, 0, 0, 00)
        #glRotatef(1, 3, 1 ,1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        cube.draw()
        pygame.display.flip()
        pygame.time.wait(10)


main()
