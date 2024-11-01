from typing import List, Optional, cast

from pythonsrc.rod_mechanics.external_forces.contact_force import ContactForce
from pythonsrc.rod_mechanics.external_forces.floor_contact_force import FloorContactForce
from pythonsrc.rod_mechanics.base_force import BaseForce


class ForceContainer:
    def __init__(self, m_forces: Optional[List[BaseForce]] = None):
        #print("HIHIHI I AM RUNNING")
        self.__forces = m_forces if m_forces is not None else []
        self.cf : ContactForce= None
        self.ff : FloorContactForce = None

    def compute_force(self,dt):
        for force in self.__forces:
            force.compute_force(dt)

    def compute_forces_and_jacobian(self, dt):
        for force in self.__forces:
            # print(force)
            force.compute_force_and_jacobian(dt)

    def setup_force_stepper_access(self, stepper):
        for force in self.__forces:
            print(force)
            force.set_time_stepper(stepper)
    
    def add_force(self,force: BaseForce):
        if self.cf == None and isinstance(force, ContactForce):
            print("HIHIH")
            self.cf = force # NEED TO TYPE CAST THIS TO A CONTACT FORCE
            print(self.cf)
        if self.ff == None and isinstance(force, ContactForce):
            self.ff = force # NEED TO TYPE CASE THIS AS A FLOOR CONTACT FORCE
        
        self.__forces.append(force)

    def get_force(self):
        return self.__forces

    