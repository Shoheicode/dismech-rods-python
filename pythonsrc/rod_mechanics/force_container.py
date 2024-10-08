from typing import List, Optional

from pythonsrc.rod_mechanics.base_force import BaseForce


class forceContainer:
    def __init__(self) -> None:
        self.cf = None
        self.ff = None
        self.__forces = None
        pass
    
    def __init__(self, m_forces: Optional[List[BaseForce]] = None):
        self.cf = None
        self.ff = None
        for force in m_forces:
            self.addForce(force)

    def compute_force(self,dt):
        for force in self.__forces:
            force.computeForce(dt)

    def compute_forces_and_jacobian(self, dt):
        for force in self.__forces:
            force.compute_force_and_jacobian(dt)

    def setup_force_stepper_access(self, stepper):
        for force in self.__forces:
            force.set_time_stepper(stepper)
    
    def add_force(self,force):
        if self.cf == None:
            cf = force # NEED TO TYPE CAST THIS TO A CONTACT FORCE
        if self.ff == None:
            ff = force # NEED TO TYPE CASE THIS AS A FLOOR CONTACT FORCE
        
        self.__forces.append(force)

    