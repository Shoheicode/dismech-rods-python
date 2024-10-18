import numpy as np

def parallel_transport(d1_1: np.ndarray, t1: np.ndarray, t2: np.ndarray, 
                         d1_2: np.ndarray = None):
        """
        Parallel transport reference frame between edges.
        
        Args:
            d1_1: First director of first frame
            t1: First tangent
            t2: Second tangent
            d1_2: Output first director of second frame
        """
        # Assuming t1, t2, and d1_1 are numpy arrays representing 3D vectors.
        
        # Cross product of t1 and t2
        b = np.cross(t1, t2)

        # Check if the norm of b is 0
        if np.linalg.norm(b) == 0:
            d1_2 = d1_1
        else:
            # Normalize b and make it orthogonal to t1 and t2
            b = b / np.linalg.norm(b)
            b = b - np.dot(b, t1) * t1
            b = b / np.linalg.norm(b)
            b = b - np.dot(b, t2) * t2
            b = b / np.linalg.norm(b)

            # Compute n1 and n2 as cross products
            n1 = np.cross(t1, b)
            n2 = np.cross(t2, b)

            # Compute d1_2 based on dot products and projections
            d1_2 = np.dot(d1_1,t1) * t2 + np.dot(d1_1,n1) * n2 + np.dot(d1_1,b) * b
        
        return d1_2

def test_parallel_transport():
  """
  This function tests the parallel_transport function by checking if the
  transported vector is orthogonal to the new tangent vector.
  """

  # Test case 1: Orthogonal tangents
  u = np.array([1, 0, 0])
  t1 = np.array([0, 1, 0])
  t2 = np.array([0, 0, 1])
  u_transported = parallel_transport(u, t1, t2)
  assert np.allclose(np.dot(u_transported, t2), 0), "Test case 1 failed"
  # Returns True if two arrays are element-wise equal within a tolerance.

  # Test case 2: Parallel tangents
  u = np.array([1, 1, 1])
  t1 = np.array([1, 0, 0])
  t2 = np.array([2, 0, 0])
  u_transported = parallel_transport(u, t1, t2)
  assert np.allclose(u_transported, u), "Test case 2 failed"

  print("All test cases passed")

test_parallel_transport()