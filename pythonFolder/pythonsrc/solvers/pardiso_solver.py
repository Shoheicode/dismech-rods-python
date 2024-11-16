import numpy as np
from pythonsrc.solvers.solver_types import SolverType
from pythonsrc.solvers.base_solver import BaseSolver
from scipy.sparse.linalg import spsolve, factorized, splu
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

        # print()

        # Cumulative sum for CSR indexing
        # ia[1:] = np.cumsum(ia[:-1]) + ia[1:]
        for i in range(n - 1):
            ia[i + 2] += ia[i + 1]
        for i in range(n):
            ia[i + 1] += 1

        # print("IA ", ia)

        # print("ia After", ia)
        ia -= 1  # Convert to 0-based indexing required by SciPy
        # ia = np.concatenate(([0], ia[:-1]))  # Fix for CSR indexing in SciPy
        # print("IA", ia)
        
        # Generate CSR format for Jacobian matrix
        non_zero_elements = sorted(self.stepper.non_zero_elements)
        # print("Non zero elements", non_zero_elements)
        ja = np.array([col + 1 for _, col in non_zero_elements], dtype=np.int32)
        a = np.array([self.stepper.Jacobian[row, col] for row, col in non_zero_elements], dtype=np.float64)

        # print("NONE ZERO ELEMENTS", len(non_zero_elements))
        # print("JACOBIAN", a[0 : 100])

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
        # ia[0]-=1
        # print(ia)
        # ia-=1

        csr_matrix_a = csc_matrix((a, ja-1, ia), shape=(n, n))
        # print("CSR MATRIX", csr_matrix_a)

        # try:
        #     # Perform symbolic factorization by converting to CSR format and analyzing structure
        #     csr_matrix_a.sort_indices()  # CSR format sorting for efficient factorization
        #     print("Reordering completed.")
        # except Exception as e:
        #     print("\nERROR during symbolic factorization:", e)

        # # Phase 22: Numerical Factorization
        # try:
        #     factorized_A = factorized(csr_matrix_a)  # Factorize the matrix
        #     print("Factorization completed.")
        # except Exception as e:
        #     print("\nERROR during numerical factorization:", e)

        # # Phase 33: Back substitution and iterative refinement
        # try:
        #     # Solve the system Ax = b using the factorized matrix
        #     x = factorized_A(self.stepper.force)
        #     print("Solution of the system:")
        #     for j in range(n):
        #         print(f"x[{j}] = {x[j]:.6f}")
        # except Exception as e:
        #     print("\nERROR during solution:", e)


        try:
            # Solve the linear system using Pardiso
            solution = spsolve(csr_matrix_a, self.stepper.force)
            self.stepper.dx = solution
            self.stepper.DX = solution
            # print("STEPPER DX", self.stepper.dx)
        except Exception as e:
            print(f"Solver error: {e}")
            self.error = 1

        # try:
        #     # Perform symbolic factorization by converting to CSR format and analyzing structure
        #     csr_matrix_a.sort_indices()  # CSR format sorting for efficient factorization
        #     print("Reordering completed.")
        # except Exception as e:
        #     print("\nERROR during symbolic factorization:", e)
        #     exit(1)
        # try:
        #     # lu = factorized(csr_matrix_a)
        #     lu = splu(csr_matrix_a)
        # except Exception as e:
        #     print("Error during matrix factorization:", e)
        #     return

        # # Solve the linear system
        # try:
        #     b = self.stepper.force
        #     # print("stepper force", b)
        #     dx = lu.solve(b)
        #     # print(spsolve(csr_matrix_a, self.stepper.force))
        #     self.stepper.dx = dx  # Save the solution back to stepper.dx
        #     self.stepper.DX = dx
        #     # print("Solution:", dx)
        # except Exception as e:
        #     print("Error during solution:", e)
