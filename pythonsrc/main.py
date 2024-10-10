# from OpenGL.GL import *
# from OpenGL.GLUT import *
# from OpenGL.GLU import *
# import numpy as np

# # Note THIS ONLY WORKS IN PYOPENGL 3.1.0

# # pip install PyOpenGL==3.1.0 use command to download it

# # Define vertices, edges, and colors for the cube
# vertices = [
#     [100, 100, 200],
#     [100, 200, 200],
#     [200, 200, 200],
#     [200, 100, 200],
#     [100, 100, 100],
#     [100, 200, 100],
#     [200, 200, 100],
#     [200, 100, 100]
# ]

# edges = [
#     (0, 1), (1, 2), (2, 3), (3, 0),
#     (4, 5), (5, 6), (6, 7), (7, 4),
#     (0, 4), (1, 5), (2, 6), (3, 7)
# ]

# surfaces = [
#     (0, 1, 2, 3),
#     (4, 5, 6, 7),
#     (0, 1, 5, 4),
#     (2, 3, 7, 6),
#     (0, 3, 7, 4),
#     (1, 2, 6, 5)
# ]

# colors = [
#     [1, 0, 0],
#     [0, 1, 0],
#     [0, 0, 1],
#     [1, 1, 0],
#     [1, 0, 1],
#     [0, 1, 1]
# ]

# #MAKES A SQUARE
# w,h= 500,500
# def square():
#     print(np.zeros(2))
#     glBegin(GL_QUADS)
#     for i, surface in enumerate(surfaces):
#         glColor3fv(colors[i])  # Color the surfaces differently
#         for vertex in surface:
#             glVertex3fv(vertices[vertex])
#     glEnd()

#     glBegin(GL_LINES)
#     for edge in edges:
#         glColor3fv([1, 1, 1])  # Color the edges white
#         for vertex in edge:
#             glVertex3fv(vertices[vertex])
#     glEnd()
#     # glutSwapBuffers()

# def iterate():
#     glViewport(0, 0, 500, 500)
#     glMatrixMode(GL_PROJECTION)
#     glLoadIdentity()
#     glOrtho(0.0, 500, 0.0, 500, 0.0, 1.0)
#     glMatrixMode (GL_MODELVIEW)
#     glLoadIdentity()

# def showScreen():
#     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
#     glLoadIdentity()
#     iterate()
#     glColor3f(1.0, 0.0, 3.0)
#     square()
#     glutSwapBuffers()

# glutInit()
# glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
# glutInitWindowSize(500, 500)
# glutInitWindowPosition(0, 0)
# wind = glutCreateWindow("OpenGL Coding Practice")
# glutDisplayFunc(showScreen)
# glutIdleFunc(showScreen)
# glutMainLoop()

from pythonsrc.world import world
import sys

my_world : world = None

