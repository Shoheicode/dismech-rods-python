import numpy as np
from pythonsrc.solvers.base_solver import BaseSolver
from scipy.sparse.linalg import spsolve, csr_matrix

class PardisoSolver(BaseSolver):
    def __init__(self, stepper):
        """
        Initialize the Pardiso solver with a given implicit time stepper.

        Parameters:
            stepper (implicitTimeStepper): The implicit time stepper associated with this solver.
        """
        super().__init__(stepper, solver_type="PARDISO_SOLVER")
        
        # Initialize Pardiso control parameters and memory pointers.
        self.pt = [None] * 64  # Memory pointer
        self.mtype = 11  # Real unsymmetric matrix
        self.maxfct = 1  # Maximum number of numerical factorizations
        self.mnum = 1    # Which factorization to use
        self.msglvl = 0  # No printout
        self.error = 0   # Initialize error flag
        self.iparm = [0] * 64
        self.iparm[0] = 1  # No solver default
        self.iparm[1] = 2  # Fill-in reordering from METIS
        self.iparm[7] = 2  # Max numbers of iterative refinement steps
        self.iparm[9] = 13 # Perturb pivot elements with 1E-13
        self.iparm[23] = 10 # Two-level factorization for nonsymmetric matrices

    def integrator(self):
        """
        Perform the integration step using the Pardiso solver.
        This method assembles the matrix in CSR format and solves the system.
        """
        n = self.stepper.freeDOF
        ia = self.stepper.ia

        # Cumulative sum for CSR indexing
        ia[1:] = np.cumsum(ia[:-1]) + ia[1:]
        
        # Generate CSR format for Jacobian matrix
        non_zero_elements = sorted(self.stepper.non_zero_elements)
        ja = np.array([col + 1 for _, col in non_zero_elements], dtype=np.int32)
        a = np.array([self.stepper.Jacobian[row, col] for row, col in non_zero_elements], dtype=np.float64)
        
        csr_matrix_a = csr_matrix((a, ja - 1, ia), shape=(n, n))

        try:
            # Solve the linear system using Pardiso
            solution = spsolve(csr_matrix_a, self.stepper.force)
            self.stepper.dx = solution
        except Exception as e:
            print(f"Solver error: {e}")
            self.error = 1
