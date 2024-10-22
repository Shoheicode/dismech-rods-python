import numpy as np

def get_ref_twist(self):
        # nv = len(refTwist)
        # self.ne = self.nv - 1
        for i in np.arange(1,self.ne):
            u0 = self.d1[i-1]
            u1 = self.d1[i]
            t0 = self.tangent[i-1]
            t =  self.tangent[i]
            ut = np.zeros(3)
            ut = self.__parallel_transport(u0,t0,t, ut)
            ut = self.__rotate_axis_angle(ut,t,self.ref_twist_old[i])
            sgnAngle = self.__signed_angle(ut,u1,t)
            self.ref_twist[i] = self.ref_twist_old[i] + sgnAngle

def test_getRefTwist():
  """
  This function tests the getRefTwist function by comparing the output
  with expected values for specific test cases.
  """

  # Test case 1: Straight rod
  d1 = np.array([[1, 0, 0], [1, 0, 0], [1, 0, 0]])
  tangent = np.array([[0, 1, 0], [0, 1, 0], [0, 1, 0]])
  refTwist = np.zeros(4)
  refTwist_calculated = getRefTwist(d1, tangent, refTwist)
  assert np.allclose(refTwist_calculated, np.zeros(4)), "Test case 1 failed"

  # Test case 2: Twisted rod (90 degrees twist)
  d1 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
  tangent = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
  refTwist = np.zeros(4)
  refTwist_calculated = getRefTwist(d1, tangent, refTwist)
  assert np.allclose(refTwist_calculated,
                      np.array([0, np.pi/2, np.pi/2, 0])), "Test case 2 failed"

  print("All test cases passed")

test_getRefTwist()