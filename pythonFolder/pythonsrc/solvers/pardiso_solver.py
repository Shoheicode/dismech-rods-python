import numpy as np
from pythonsrc.solvers.solver_types import SolverType
from pythonsrc.solvers.base_solver import BaseSolver
from scipy.sparse.linalg import spsolve, splu
from scipy.sparse import csr_matrix, csc_matrix

class PardisoSolver(BaseSolver):
    def __init__(self, stepper):
        """
        Initialize the Pardiso solver with a given implicit time stepper.

        Parameters:
            stepper (implicitTimeStepper): The implicit time stepper associated with this solver.
        """
        super().__init__(stepper, SolverType.PARDISO_SOLVER)
        
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
        # print("stepper before", self.stepper.ia)
        # print(len(self.stepper.ia))
        
        ia = np.array(self.stepper.ia, copy = True)

        # print(ia)

        # Cumulative sum for CSR indexing
        # ia[1:] = np.cumsum(ia[:-1]) + ia[1:]
        for i in range(n - 1):
            ia[i + 2] += ia[i + 1]
        for i in range(n):
            ia[i + 1] += 1

        print("ia After", ia)
        
        # Generate CSR format for Jacobian matrix
        non_zero_elements = sorted(self.stepper.non_zero_elements)
        print("Non zero elements", non_zero_elements)
        ja = np.array([col + 1 for _, col in non_zero_elements], dtype=np.int32)
        a = np.array([self.stepper.Jacobian[row, col] for row, col in non_zero_elements], dtype=np.float64)

        # print("NONE ZERO ELEMENTS", len(non_zero_elements))
        # print(non_zero_elements)

        # print(non_zero_elements)
        
        # for i in range(50):
        #     print("JA VALUE",i, ja[i])

        # for i in range(50):
        #     print("A VALUE",i, a[i])
        # print(ja - 1)
        # print("HELLOOOOOO")
        # print(ia)
        # ia -=1
        # ia = np.concatenate(([0], ia))
        # print("print ia", ia)
        # ia = ia[:-1]
        # print("FORCE VALUES: ", self.stepper.force)
        # ia[0] = 0
        # ia = ia[:-1]
        # ia -=1

        csr_matrix_a = csc_matrix((a, ja-1, ia-1), shape=(n, n))

        # # Assuming `A` is your sparse matrix
        # if np.linalg.cond(csr_matrix_a.toarray()) > 1 / np.finfo(csr_matrix_a.dtype).eps:
        #     print("Matrix is ill-conditioned")

        # csr_matrix_a = csc_matrix(csr_matrix_a)

        # if np.linalg.cond(csr_matrix_a.toarray()) > 1 / np.finfo(csr_matrix_a.dtype).eps:
        #     print("Matrix is ill-conditioned")

        # csr_matrix_a = csr_matrix(csr_matrix_a)


        # try:
        #     # Solve the linear system using Pardiso
        #     solution = spsolve(csr_matrix_a, self.stepper.force)
        #     self.stepper.dx = solution
        #     print("STEPPER DX", self.stepper.dx)
        # except Exception as e:
        #     print(f"Solver error: {e}")
        #     self.error = 1
        try:
            lu = splu(csr_matrix_a)
        except Exception as e:
            print("Error during matrix factorization:", e)
            return

        # Solve the linear system
        try:
            b = self.stepper.force
            dx = lu.solve(b)
            # print(spsolve(csr_matrix_a, self.stepper.force))
            self.stepper.dx = dx  # Save the solution back to stepper.dx
            self.stepper.DX = dx
            # print("Solution:", dx)
        except Exception as e:
            print("Error during solution:", e)
