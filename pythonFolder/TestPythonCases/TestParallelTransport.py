import numpy as np

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