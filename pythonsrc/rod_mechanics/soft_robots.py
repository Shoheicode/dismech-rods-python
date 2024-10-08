class softRobots():
    def __init__(self):
        pass

    def addLimb(start, end, num_nodes, rho, rod_radius, youngs_modulus, poisson_ratio, mu = 0.0):
        pass

    def addLimb(nodes, rho, rod_radius, youngs_modulus, poisson_ratio, mu = 0.0):
        pass

    def createJoint(limb_idx, node_idx):
        pass

    def addToJoint(joint_idx, limb_idx, node_idx):
        pass

    def lockEdge(limb_idx, edge_idx):
        pass

    def applyInitialVelocities(limb_idx, velocities):
        pass

    def setup():
        pass

    # void addController(const shared_ptr<baseController>& controller);
    def addController(controller):
        pass

    
