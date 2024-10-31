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

from pythonsrc.globalDefinitions import RenderParams, SimParams, RenderEngine
from pythonsrc.cantlieverExample import get_robot_description
from pythonsrc.logging.worldLogger import WorldLogger
from pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
from pythonsrc.simulation_environment.derSimulationEnvironment import derSimulationEnvironment
from pythonsrc.simulation_environment.headlessDERSimulationEnvironment import HeadlessDERSimulationEnvironment
from pythonsrc.world import world
import sys

my_world : world = None

def main():
    soft_robots = SoftRobots()
    forces = ForceContainer()
    sim_params = SimParams()
    render_params = RenderParams(RenderEngine.HEADLESS)

    logger : WorldLogger = None

    lis = get_robot_description(None, sys.argv, soft_robots, forces, logger, sim_params, render_params)

    print(lis)

    soft_robots.setup()

    # my_world = world(soft_robots, forces, sim_params)

    # env : derSimulationEnvironment = None

    # match render_params.renderer:
    #     case RenderEngine.HEADLESS:
    #         print("HIHIHI")
    #         env = HeadlessDERSimulationEnvironment(my_world, render_params, logger)
    #     case RenderEngine.OPENGL:
    #         print("OPTION 2 selected")
    #         return "Option 2 selected"
    #     case _:
    #         raise RuntimeError("Unknown renderer type provided")
        
    # env.runSimulation()

    exit(0)
    
if __name__ == "__main__":
    main()