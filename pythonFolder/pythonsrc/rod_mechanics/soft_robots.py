from typing import List

from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.rod_mechanics.elasticjoint import ElasticJoint


class SoftRobots():
    def __init__(self):
        self.limbs = List[ElasticRod]
        self.joints = List[ElasticJoint]
        self.controllers = []

    def addLimb(self, start, end, num_nodes, rho, rod_radius, youngs_modulus, poisson_ratio, mu = 0.0):
        self.limbs.append("hi")

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


