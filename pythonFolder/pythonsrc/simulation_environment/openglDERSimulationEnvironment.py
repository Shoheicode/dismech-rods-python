import sys
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from typing import Optional
import numpy as np
from pythonsrc.simulation_environment.derSimulationEnvironment import derSimulationEnvironment
from pythonsrc.world import world
from pythonsrc.globalDefinitions import RenderParams
from pythonsrc.logging.worldLogger import WorldLogger

class OpenGLDERSimulationEnvironment(derSimulationEnvironment):
    """
    Class for OpenGL DER Simulation Environment.
    """

    # Class-level attributes (similar to static variables)
    opengl_world = None
    opengl_logger = None
    opengl_is_logging = False
    opengl_cmdline_per = 0
    show_mat_frames = False
    render_scale = 1.0

    def __init__(self, m_world: world, render_params: RenderParams, logger:WorldLogger, argv = []):
        """
        Initialize the OpenGL simulation environment.
        Args:
            m_world: A shared pointer or instance to the world object.
            render_params: Rendering parameters.
            logger: A shared pointer or instance of the worldLogger.
            argv: Command-line arguments.
        """
        super().__init__(m_world, render_params, logger)
        self.opengl_world = m_world
        self.opengl_logger = logger
        self.opengl_is_logging = logger is not None
        self.opengl_cmdline_per = render_params.cmd_line_per
        self.render_scale = render_params.render_scale
        self.show_mat_frames = render_params.show_mat_frames

        # Initialize GLUT
        glutInit(argv)
        glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB)
        glutInitWindowSize(1500, 1200)
        glutInitWindowPosition(100, 100)
        glutCreateWindow(b"disMech")

        # Configure OpenGL settings
        glClearColor(0.7, 0.7, 0.7, 0.0)
        glClearDepth(10.0)
        glShadeModel(GL_SMOOTH)

        glLoadIdentity()
        gluLookAt(0.05, 0.05, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
        glPushMatrix()

        # Set up callbacks
        glutKeyboardFunc(self.key_handler)
        glutDisplayFunc(self.der_opengl_display)

        # Log initial state if logging is enabled
        if self.opengl_is_logging:
            self.opengl_logger.log_world_data()

    def runSimulation(self):
        """
        Run the simulation.
        """
        while self.opengl_world.simulation_runing():
            self.step_simulation()

    def step_simulation(self):
        """
        Perform a single step of the simulation.
        """
        glutMainLoopEvent()
        glutPostRedisplay()
        self.cmdlineOutputHelper()

    def der_opengl_display(self):
        """
        Main display function for OpenGL. Handles simulation and rendering.
        """
        try:
            self.opengl_world.update_time_step()
        except RuntimeError as ex:
            print(f"Caught a runtime error during world update: {ex}")
            print("Attempting clean shutdown...")
            self.clean_shutdown()
            glutLeaveMainLoop()
            return

        # Log world data if logging is enabled
        if self.opengl_is_logging:
            self.opengl_logger.log_world_data()

        # Clear screen and Z-buffer
        glClear(GL_COLOR_BUFFER_BIT)

        # Draw axes
        self.draw_axes()

        # Draw the soft robot limbs and joints
        self.draw_limbs_and_joints()

        # Draw material directors if enabled
        if self.show_mat_frames:
            self.draw_material_directors()

        # Draw nodes
        self.draw_nodes()

        glFlush()
    
    def draw_limbs_and_joints(self):
        """
        Draws the limbs and joints of the soft robot.
        """
        glLineWidth(4.0)
        glBegin(GL_LINES)

        # Draw limbs
        glColor3f(0.0, 0.0, 0.0)
        for limb_idx, limb in enumerate(self.opengl_world.soft_robots.limbs):
            for i in range(limb.ne):
                if not limb.is_edge_joint[i]:
                    self.draw_line_between_nodes(limb_idx, i, i + 1)

        # Draw joints
        glColor3f(0.0, 0.0, 1.0)
        for joint in self.opengl_world.soft_robots.joints:
            for conn in joint.connected_nodes:
                n, l = conn
                glVertex3f(*self.get_scaled_coordinates(n, l))
                glVertex3f(*joint.x * self.render_scale)

        glEnd()

    def draw_material_directors(self):
        """
        Draws material directors for visualization.
        """
        glLineWidth(2.5)
        glBegin(GL_LINES)

        for limb_idx, limb in enumerate(self.opengl_world.soft_robots.limbs):
            for i in range(limb.ne):
                if not limb.is_edge_joint[i]:
                    midpoint = self.get_midpoint_between_nodes(limb_idx, i, i + 1)
                    m1 = 0.05 * np.array(self.opengl_world.get_M1(i, limb_idx))
                    m2 = 0.05 * np.array(self.opengl_world.get_M2(i, limb_idx))

                    glColor3f(1.0, 0.0, 0.0)
                    self.draw_vector(midpoint, m1)
                    glColor3f(0.5, 0.0, 1.0)
                    self.draw_vector(midpoint, m2)

        glEnd()

    def draw_nodes(self):
        """
        Draws the nodes of the soft robot.
        """
        glPointSize(4.0)
        glBegin(GL_POINTS)
        glColor3f(0.0, 1.0, 0.0)

        for limb_idx, limb in enumerate(self.opengl_world.soft_robots.limbs):
            for i in range(limb.nv):
                glVertex3f(*self.get_scaled_coordinates(i, limb_idx))

        glEnd()
    
    def draw_axes(self):
        """
        Draws the coordinate axes.
        """
        axis_len = 1.0
        glLineWidth(0.5)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)  # X-axis
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(axis_len, 0.0, 0.0)
        glColor3f(0.0, 1.0, 0.0)  # Y-axis
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, axis_len, 0.0)
        glColor3f(0.0, 0.0, 1.0)  # Z-axis
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, axis_len)
        glEnd()

    def draw_line_between_nodes(self, limb_idx, node1, node2):
        """
        Draws a line between two nodes.
        """
        glVertex3f(*self.get_scaled_coordinates(node1, limb_idx))
        glVertex3f(*self.get_scaled_coordinates(node2, limb_idx))

    def get_scaled_coordinates(self, node_idx, limb_idx):
        """
        Returns scaled coordinates of a node.
        """
        return [
            self.opengl_world.get_coordinate(4 * node_idx + i, limb_idx) * self.render_scale
            for i in range(3)
        ]

    def get_midpoint_between_nodes(self, limb_idx, node1, node2):
        """
        Returns the midpoint between two nodes.
        """
        return np.mean(
            [self.get_scaled_coordinates(node1, limb_idx), self.get_scaled_coordinates(node2, limb_idx)],
            axis=0
        )

    def draw_vector(self, origin, vector):
        """
        Draws a vector from an origin point.
        """
        glVertex3f(*origin)
        glVertex3f(*(origin + vector))

    def key_handler(self, key, x, y):
        """
        Handles keypresses in the OpenGL window.
        """
        if key == b'\x1b':  # ESC key
            sys.exit(0)