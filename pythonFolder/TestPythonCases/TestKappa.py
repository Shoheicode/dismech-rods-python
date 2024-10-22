from typing import List
import numpy as np

def compute_kappa(node0 = None,node1 = None,node2 = None,m1e = None,m2e = None,m1f = None,m2f = None):
    #CODE THAT I IMPLEMENTED. ADJUSTED TO WORK WITH GIVEN TEST CASES

    # # First loop: Compute kb using the tangent vectors
    # for i in range(1, self.ne):
    #     t0 = self.tangent[i - 1, :]  # Get the (i-1)th row of the tangent array
    #     t1 = self.tangent[i, :]      # Get the ith row of the tangent array
    #     self.kb[i, :] = 2.0 * np.cross(t0, t1) / (1.0 + np.dot(t0, t1))
    # # Second loop: Compute kappa using m1, m2, and kb
    # for i in range(1, self.ne):
    #     m1e = self.m1[i - 1, :]  # Get the (i-1)th row of m1
    #     m2e = self.m2[i - 1, :]  # Get the (i-1)th row of m2
    #     m1f = self.m1[i, :]      # Get the ith row of m1
    #     m2f = self.m2[i, :]      # Get the ith row of m2

    #     # Calculate the values for kappa
    #     self.kappa[i, 0] = 0.5 * np.dot(self.kb[i, :], (m2e + m2f))  # First component of kappa
    #     self.kappa[i, 1] = -0.5 * np.dot(self.kb[i, :], (m1e + m1f))  # Second component of kappa
    
    nodes:List[np.ndarray] = [node0, node1, node2]
    # print("NODES", nodes)
    nv = len(nodes)
    ne = nv-1
    tangent = np.zeros((ne, 3))
    kb = np.zeros((nv, 3))
    kappa = np.zeros((nv, 2))
    ndof = len(nodes) * 4 - 1

    x: np.ndarray = np.zeros(ndof)   # Current timestep DOFs

    for i in range(nv):
        x[4*i] = nodes[i][0]
        x[4*i+1] = nodes[i][1]
        x[4*i+2] = nodes[i][2]
        if i < nv - 1:
            x[4*i+3] = 0
    
    # print("X VALUE")
    # print(x)

    # From tangent code
    def compute_tangent():
        for i in range(ne):
            # Extract segments (3 elements) from 'x' to compute the tangent vector.
            tangent[i, :] = x[4 * (i + 1): 4 * (i + 1) + 3] - x[4 * i: 4 * i + 3]

            # Normalize the tangent vector.
            tangent[i, :] = tangent[i, :] / np.linalg.norm(tangent[i, :])
    
    compute_tangent()
    # print("TANGENT: ")
    # print(tangent)
    global t0,t1

    for i in range(1, ne):
        t0 = tangent[i - 1, :]  # Get the (i-1)th row of the tangent array
        t1 = tangent[i, :]      # Get the ith row of the tangent array
        kb[i, :] = 2.0 * np.cross(t0, t1) / (1.0 + np.dot(t0, t1))
        #print("CROSS1:",kb[i,:])

    # t2 = (node1 - node0) / np.linalg.norm(node1 - node0)
    # t3 = (node2 - node1) / np.linalg.norm(node2 - node1)
    # print("CROSS2:", 2.0 * np.cross(t2,t3) / (1.0 + np.dot(t2,t3)))


    # print("T0: ",t0)
    # print("T1: ", t1)
    # print("KB: ", kb)
    
    # # Second loop: Compute kappa using m1, m2, and kb
    for i in range(1, ne):

        # Calculate the values for kappa
        #print(kb[i,:])
        kappa[i, 0] = 0.5 * np.dot(kb[i, :], (m2e + m2f))  # First component of kappa
        #print("KAPPA:", kappa[i, 0])
        kappa[i, 1] = -0.5 * np.dot(kb[i, :], (m1e + m1f))  # Second component of kappa

    # print(kappa)
    # print(kappa[1])
    return kappa[1] #Because based on the C++ code, this is what is implemented

# Test works
def test_computekappa():
  """
  This function tests the computekappa function by comparing the output with
  expected values for a specific test case.
  """

  # Test case 1: Straight rod with no curvature
  node0 = np.array([0, 0, 0])
  node1 = np.array([1, 0, 0])
  node2 = np.array([2, 0, 0])
  m1e = np.array([0, 1, 0])
  m2e = np.array([0, 0, 1])
  m1f = np.array([0, 1, 0])
  m2f = np.array([0, 0, 1])

  kappa_calculated = compute_kappa(node0, node1, node2, m1e, m2e, m1f, m2f)

  # Expected output: zero curvature for a straight rod
  kappa_expected = np.array([0, 0])

  # Check if the calculated curvature is close to the expected curvature
  assert np.allclose(kappa_calculated, kappa_expected), "Test case failed"

  # Test case 2: A rod with inifinite curvature (180 degrees turn)
  node0 = np.array([0, 0, 0])
  node1 = np.array([1, 0, 0])
  node2 = np.array([0, 0, 0])
  m1e = np.array([0, 1, 0])
  m2e = np.array([0, 0, 1])
  m1f = np.array([0, 1, 1])
  m2f = np.array([0, 0, -1])

  kappa_calculated = compute_kappa(node0, node1, node2, m1e, m2e, m1f, m2f)

  # Check for NaN values
  assert np.isnan(kappa_calculated[0]), "NaN curvature check 1 failed"
  assert np.isnan(kappa_calculated[1]), "NaN curvature check 2 failed"

  print("Test case passed")

test_computekappa()