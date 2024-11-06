from pythonsrc.rod_mechanics.base_force import BaseForce

class InertialForce(BaseForce):
    def __init__(self, soft_robots):
        super().__init__(soft_robots)
        self.f = 0.0
        self.jac = 0.0

    def compute_force(self, dt):
        # print("STEPPER VALUE: ",self.stepper)
        limb_idx = 0
        for limb in self.soft_robots.limbs:
            # print("LIMB MASS ARRAY",limb.mass_array)
            for i in range(limb.ndof):
                if limb.is_dof_joint[i]:
                    print("SKIP THE JOINT")
                    continue
                # Compute force based on mass and velocities
                self.f = (limb.mass_array[i] / dt) * ((limb.x[i] - limb.x0[i]) / dt - limb.u[i])
                # if(i < 50):
                    # print("limb VALUE", i , ":", limb.x[i])
                    # print("Prev limb value", i, ":", limb.x0[i])
                #     print("LIMB TEST", i , ": ", limb.x[i] - limb.x0[i])
                # print(limb.x0)
                
                self.stepper.add_force(i, self.f, limb_idx)
            limb_idx += 1

        for joint in self.soft_robots.joints:
            for i in range(3):
                # Compute joint force similarly
                self.f = (joint.mass / dt) * ((joint.x[i] - joint.x0[i]) / dt - joint.u[i])
                self.stepper.add_force(4 * joint.joint_node + i, self.f, joint.joint_limb)

    def compute_force_and_jacobian(self, dt):
        # print("INTERIAL FORCE BASE FORCE")
        # Re-use compute_force function to calculate forces
        # print("DT VALUE",dt)
        self.compute_force(dt)
        # print(self.stepper)

        limb_idx = 0
        for limb in self.soft_robots.limbs:
            for i in range(limb.ndof):
                if limb.is_dof_joint[i]:
                    continue
                # Compute jacobian based on the mass array and timestep
                self.jac = limb.mass_array[i] / (dt * dt)
                self.stepper.add_jacobian(i, i, self.jac, limb_idx)
            limb_idx += 1

        for joint in self.soft_robots.joints:
            ind = 4 * joint.joint_node
            for i in range(3):
                # Compute joint jacobian similarly
                self.jac = joint.mass / (dt * dt)
                self.stepper.add_jacobian(ind + i, ind + i, self.jac, joint.joint_limb)
