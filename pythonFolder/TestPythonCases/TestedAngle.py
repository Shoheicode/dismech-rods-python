import numpy as np

def test_signedAngle():
  """
  This function tests the signedAngle function with three test cases.
  """
  # Test case 1: Orthogonal vectors
  u = np.array([1, 0, 0])
  v = np.array([0, 1, 0])
  n = np.array([0, 0, 1])
  angle = signedAngle(u, v, n)
  assert np.isclose(angle, np.pi/2), "Test case 1 failed"

  # Test case 2: Parallel vectors
  u = np.array([1, 1, 1])
  v = np.array([2, 2, 2])
  n = np.array([0, 1, 0])
  angle = signedAngle(u, v, n)
  assert np.isclose(angle, 0), "Test case 2 failed"

  # Test case 3: Anti-parallel vectors
  u = np.array([1, 1, 1])
  v = np.array([-1, -1, -1])
  n = np.array([0, 1, 0])
  angle = signedAngle(u, v, n)
  assert np.isclose(angle, np.pi), "Test case 3 failed"

  print("All test cases passed")

test_signedAngle()