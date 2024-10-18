import numpy as np

def rotate_axis_angle(v: np.ndarray, z: np.ndarray, theta: float):
        """
        Rotate vector v around axis z by angle theta.
        
        Args:
            v: Vector to rotate
            z: Rotation axis
            theta: Rotation angle
        """
        if theta == 0:
            return v
        else: 
            c = np.cos(theta)
            s = np.sin(theta)
            v[:] = v * c + np.cross(z, v) * s + z * np.dot(z, v) * (1 - c)
            return v[:]

def test_rotateAxisAngle():
  """
  This function tests the rotateAxisAngle function by comparing the output with
  the expected output for a given set of inputs.
  """

  # Test case: Rotate a vector by 90 degrees around the z-axis
  v = np.array([1, 0, 0])
  axis = np.array([0, 0, 1])
  theta = np.pi/2
  v_rotated = rotateAxisAngle(v, axis, theta)

  # Expected output
  v_expected = np.array([0, 1, 0])

  # Check if the output is close to the expected output
  assert np.allclose(v_rotated, v_expected), "Test case failed"

  print("Test case passed")

test_rotateAxisAngle()