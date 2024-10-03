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

    def computeForce(self,dt):
        for force in self.__forces:
            force.computeForce(dt)

    def computeForcesAndJacobian(self, dt):
        for force in self.__forces:
            force.computeForceAndJacobian(dt)

    def setupForceStepperAccess(self, stepper):
        for force in self.__forces:
            force.setTimeStepper(stepper)
    
    def addForce(self,force):
        if self.cf == None:
            cf = force # NEED TO TYPE CAST THIS TO A CONTACT FORCE
        if self.ff == None:
            ff = force # NEED TO TYPE CASE THIS AS A FLOOR CONTACT FORCE
        
        self.__forces.append(force)

    