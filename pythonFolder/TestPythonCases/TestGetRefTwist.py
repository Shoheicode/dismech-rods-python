import numpy as np

def signed_angle(u: np.ndarray,v: np.ndarray, n: np.ndarray):
        w = np.cross(u,v)
        angle = np.arctan2( np.linalg.norm(w), np.dot(u,v) )
        if (np.dot(n,w) < 0):
            angle = -angle

        return angle

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

def get_ref_twist(d1 : np.ndarray, tangent, refTwist = None):
        # nv = len(refTwist)
        # self.ne = self.nv - 1

        # MY CODE FOR GET_REF. Wil adjust but still very similar implementation
        # for i in np.arange(1,self.ne):
        #     u0 = self.d1[i-1]
        #     u1 = self.d1[i]
        #     t0 = self.tangent[i-1]
        #     t =  self.tangent[i]
        #     ut = np.zeros(3)
        #     ut = self.__parallel_transport(u0,t0,t, ut)
        #     ut = self.__rotate_axis_angle(ut,t,self.ref_twist_old[i])
        #     sgnAngle = self.__signed_angle(ut,u1,t)
        #     self.ref_twist[i] = self.ref_twist_old[i] + sgnAngle

        nv = len(refTwist)
        ne = nv-1

        for i in np.arange(1, ne):
            u0 = d1[i-1]
            u1 = d1[i]
            t0 = tangent[i-1]
            t =  tangent[i]
            ut = np.zeros(3)
            ut = parallel_transport(u0,t0,t, ut)
            ut = rotate_axis_angle(ut,t, refTwist[i])
            sgnAngle = signed_angle(ut,u1,t)
            refTwist[i] = refTwist[i] + sgnAngle
        
        return refTwist




        

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