from enum import Enum
import numpy as np

class IntegratorMethod(Enum):
    FORWARD_EULER = "FORWARD_EULER"
    VERLET_POSITION = "VERLET_POSITION"
    BACKWARD_EULER = "BACKWARD_EULER"
    IMPLICIT_MIDPOINT = "IMPLICIT_MIDPOINT"

class RenderEngine(Enum):
    HEADLESS = "HEADLESS"
    OPENGL = "OPENGL"
    MAGNUM = "MAGNUM"

class SimParams:
    def __init__(self, 
                 sim_time: float = 10.0, 
                 dt: float = 1e-3, 
                 integrator: IntegratorMethod = IntegratorMethod.BACKWARD_EULER, 
                 dtol: float = 1e-2, 
                 ftol: float = 1e-4, 
                 num_iters: int = 500, 
                 terminate_at_max: bool = True, 
                 line_search: bool = True, 
                 adaptive_time_stepping: int = 0, 
                 enable_2d_sim: bool = False):
        """
        Parameters for the simulation.
        :param sim_time: Total time for the simulation in seconds.
        :param dt: Time step size in seconds.
        :param integrator: Numerical integration scheme.
        :param dtol: Dynamics tolerance in m/s.
        :param ftol: Force tolerance.
        :param num_iters: Maximum number of iterations for a time step in implicit schemes.
        :param terminate_at_max: Whether to terminate after max iterations.
        :param line_search: Whether to enable line search for implicit schemes.
        :param adaptive_time_stepping: Enable adaptive time stepping, 0 to disable.
        :param enable_2d_sim: Enable 2D simulation, locking y and theta DOFs.
        """
        self.sim_time = sim_time
        self.dt = dt
        self.integrator = integrator
        self.dtol = dtol
        self.ftol = ftol
        self.max_iters = {
            "num_iters": num_iters,
            "terminate_at_max": terminate_at_max
        }
        self.line_search = line_search
        self.adaptive_time_stepping = adaptive_time_stepping
        self.enable_2d_sim = enable_2d_sim

class RenderParams:
    """
    Parameters for rendering.
    """
    def __init__(self, 
                 renderer: RenderEngine = RenderEngine.OPENGL, 
                 render_scale: float = 1.0, 
                 cmd_line_per: int = 1, 
                 render_per: int = 1, 
                 render_record_path: str = '', 
                 show_mat_frames: bool = False, 
                 debug_verbosity: int = 1):
        """
        Parameters for rendering.
        :param renderer: Renderer type.
        :param render_scale: Scale factor applied to Cartesian DOFs before rendering.
        :param cmd_line_per: Command line simulation info output period.
        :param render_per: Rendering period for frames.
        :param render_record_path: Path for recording frames as PNGs.
        :param show_mat_frames: Whether to show material frames (for OpenGL).
        :param debug_verbosity: Debug verbosity level.
        """
        self.renderer = renderer
        self.render_scale = render_scale
        self.cmd_line_per = cmd_line_per
        self.render_per = render_per
        self.render_record_path = render_record_path
        self.show_mat_frames = show_mat_frames
        self.debug_verbosity = debug_verbosity