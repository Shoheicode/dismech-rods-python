import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

verticies = (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
    )

edges = (
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

# Function to render the XYZ axes
def draw_axes():
    glLineWidth(2.0)
    glBegin(GL_LINES)

    # X-axis (Red)
    glColor3f(1, 0, 0)  # Red color
    glVertex3f(0, 0, 0)  # Starting point
    glVertex3f(1, 0, 0)  # End point

    # Y-axis (Green)
    glColor3f(0, 1, 0)  # Green color
    glVertex3f(0, 0, 0)  # Starting point
    glVertex3f(0, 1, 0)  # End point

    # Z-axis (Blue)
    glColor3f(0, 0, 1)  # Blue color
    glVertex3f(0, 0, 0)  # Starting point
    glVertex3f(0, 0, 1)  # End point

    glEnd()

def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])
    glEnd()


def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(0.0,0.0, -5)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # glRotatef(1, 3, 1, 1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        Cube()
        draw_axes()
        pygame.display.flip()
        pygame.time.wait(10)


main()