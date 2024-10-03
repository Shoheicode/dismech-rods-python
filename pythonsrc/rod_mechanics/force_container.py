class forceContainer:
    def __init__(self) -> None:
        self.cf = None
        self.ff = None
        self.__forces = None
        pass
    
    def __init__(self, m_forces):
        self.cf = None
        self.ff = None
        for force in m_forces:
            self.addForce(force)
        pass

    def computeForce(self,dt):
        pass

    def computeForcesAndJacobian(dt):
        pass

    def setupForceStepperAccess(stepper):
        pass
    
    def addForce(force):
        pass

    