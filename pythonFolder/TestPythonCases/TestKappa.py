import numpy as np

def compute_kappa(node0 = None,node1 = None,node2 = None,m1e = None,m2e = None,m1f = None,m2f = None):
        # First loop: Compute kb using the tangent vectors
        for i in range(1, self.ne):
            t0 = self.tangent[i - 1, :]  # Get the (i-1)th row of the tangent array
            t1 = self.tangent[i, :]      # Get the ith row of the tangent array
            self.kb[i, :] = 2.0 * np.cross(t0, t1) / (1.0 + np.dot(t0, t1))

        # Second loop: Compute kappa using m1, m2, and kb
        for i in range(1, self.ne):
            m1e = self.m1[i - 1, :]  # Get the (i-1)th row of m1
            m2e = self.m2[i - 1, :]  # Get the (i-1)th row of m2
            m1f = self.m1[i, :]      # Get the ith row of m1
            m2f = self.m2[i, :]      # Get the ith row of m2

            # Calculate the values for kappa
            self.kappa[i, 0] = 0.5 * np.dot(self.kb[i, :], (m2e + m2f))  # First component of kappa
            self.kappa[i, 1] = -0.5 * np.dot(self.kb[i, :], (m1e + m1f))  # Second component of kappa
