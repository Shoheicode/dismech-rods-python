from pythonsrc.time_stepper.implicit_time_stepper import ImplicitTimeStepper

class BaseSolver:
    def __init__(self, stepper: ImplicitTimeStepper, solver_type:str):
        """
        Initialize the base solver with a time stepper and solver type.
        
        Parameters:
            stepper (implicitTimeStepper): The implicit time stepper associated with this solver.
            solver_type (solverType): The type of solver (e.g., PARDISO, DGBSV).
        """
        self.stepper = stepper
        self.solver_type = solver_type
        self.nrhs = 1  # Typically for solving single right-hand side problems

    def integrator(self):
        """
        Abstract method to perform integration. 
        This should be implemented in subclasses.
        """
        raise NotImplementedError("Integrator method must be implemented by subclass.")

    def get_solver_type(self):
        """
        Returns the type of solver.
        
        Returns:
            solverType: The solver type of the current instance.
        """
        return self.solver_type
