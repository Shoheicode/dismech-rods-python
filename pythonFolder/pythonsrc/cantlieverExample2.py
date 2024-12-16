import numpy as np

from pythonsrc.rod_mechanics.external_forces.gravity_force import GravityForce
from pythonsrc.globalDefinitions import IntegratorMethod, RenderParams, SimParams
from pythonsrc.logging.rod_node_logger import RodNodeLogger
from pythonsrc.logging.worldLogger import WorldLogger
from pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonsrc.rod_mechanics.soft_robots import SoftRobots

def load_txt(filepath):
    return np.loadtxt(filepath)

def convert_float_to_scientific_str(value):
    return f"{value:.6e}"  # Adjust precision if needed

# Main function to define the robot description
def get_robot_description(argc, argv,soft_robots:SoftRobots, forces: ForceContainer, logger: WorldLogger, sim_params: SimParams, render_params: RenderParams):

    sim_params.dt = 5e-3
    sim_params.sim_time =10.0
    sim_params.dtol = 1e-3
    sim_params.enable_2d_sim = True
    sim_params.integrator = IntegratorMethod.IMPLICIT_MIDPOINT

    render_params.show_mat_frames = True

    n = 3
    radius = 0.02
    young_mod = 1e5
    density = 500
    poisson = 0.5

    # Create a beam along the x-y plane
    soft_robots.add_limb(np.array([0.0, 0.0, 0.75]), np.array([1.0, 0.0, 0.0]), n, density, radius, young_mod, poisson)

    # ("JOINTS",soft_robots.joints)

    # Fix one end
    soft_robots.lockEdge(0, 0)

    # # Read initial velocity values
    velocities = load_txt("examples/cantilever_case/cantilever_init_velocity_n=201.txt")

    # # ("VEL", velocities)

    # # Apply the velocities
    # soft_robots.applyInitialVelocities(0, velocities)

    gravityVec = [0.0,0.0,-9.8]
    forces.add_force(GravityForce(soft_robots, gravityVec))

    # Set logger to record nodes
    logfile_base = "log_files/cantileve"
    logging_period = 1
    a = convert_float_to_scientific_str(young_mod)
    logger = RodNodeLogger(logfile_base, a, 'log_output.txt', logging_period)

    return [soft_robots, forces,sim_params, render_params, logger]
