import numpy as np

from pythonFolder.pythonsrc.globalDefinitions import RenderParams, SimParams
from pythonFolder.pythonsrc.logging.worldLogger import WorldLogger
from pythonFolder.pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonFolder.pythonsrc.rod_mechanics.soft_robots import SoftRobots

def load_txt(filepath):
    return np.loadtxt(filepath)

# Main function to define the robot description
def get_robot_description(soft_robots:SoftRobots, forces: ForceContainer, logger: WorldLogger, sim_params: SimParams, render_params: RenderParams):

    sim_params.dt = 5e-2
    sim_params.sim_time = 100
    sim_params.dtol = 1e-3
    sim_params.enable_2d_sim = True
    sim_params.integrator = 'IMPLICIT_MIDPOINT'

    render_params.show_mat_frames = True

    n = 201
    radius = 0.02
    young_mod = 1e5
    density = 500
    poisson = 0.5

    # Create a beam along the x-y plane
    soft_robots.addLimb(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), n, density, radius, young_mod, poisson)

    # Fix one end
    soft_robots.lockEdge(0, 0)

    # Read initial velocity values
    velocities = load_txt("examples/cantilever_case/cantilever_init_velocity_n=201.txt")

    # Apply the velocities
    soft_robots.applyInitialVelocities(0, velocities)

    # Set logger to record nodes
    logfile_base = "log_files/cantilever"
    logging_period = 1
    logger = RodNodeLogger(logfile_base, convert_float_to_scientific_str(young_mod), 'log_output.txt', logging_period)
