from typing import List
from pythonsrc.rod_mechanics.base_force import BaseForce
from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
import numpy as np

class elasticBendingForce(BaseForce):
    ci = 0
    chi = 0.0
    len = 0.0
    kappa1 = 0.0
    kappa2 = 0.0
    norm_e = 0.0
    norm_f = 0.0
    norm2_e = 0.0
    norm2_f = 0.0

    # Initialize vectors as numpy arrays
    t0 = np.zeros(3)
    t1 = np.zeros(3)
    te = np.zeros(3)
    tf = np.zeros(3)
    d1e = np.zeros(3)
    d1f = np.zeros(3)
    d2e = np.zeros(3)
    d2f = np.zeros(3)
    tilde_t = np.zeros(3)
    tilde_d1 = np.zeros(3)
    tilde_d2 = np.zeros(3)
    Dkappa1De = np.zeros(3)
    Dkappa1Df = np.zeros(3)
    Dkappa2De = np.zeros(3)
    Dkappa2Df = np.zeros(3)
    kbLocal = np.zeros(3)
    kappaL = np.zeros(2)
    tmp = np.zeros(3)

    D2kappa1DeDthetae = np.zeros(3)
    D2kappa1DeDthetaf = np.zeros(3)
    D2kappa1DfDthetae = np.zeros(3)
    D2kappa1DfDthetaf = np.zeros(3)

    D2kappa2DeDthetae = np.zeros(3)
    D2kappa2DeDthetaf = np.zeros(3)
    D2kappa2DfDthetae = np.zeros(3)
    D2kappa2DfDthetaf = np.zeros(3)

    m1e = np.zeros(3)
    m2e = np.zeros(3)
    m1f = np.zeros(3)
    m2f = np.zeros(3)

    # Initialize matrices
    kappa11 = np.zeros((3, 3))
    kappa22 = np.zeros((3, 3))
    f = np.zeros(3)

    # Lists of matrices (using numpy arrays instead of Eigen::MatrixXd)
    gradKappa1s: List[np.ndarray] = []
    gradKappa2s: List[np.ndarray] = []

    gradKappa1 = np.zeros((3, 3))
    gradKappa2 = np.zeros((3, 3))
    relevantPart = np.zeros((3, 3))
    kappa = np.zeros((3, 3))
    DDkappa1 = np.zeros((3, 3))
    DDkappa2 = np.zeros((3, 3))
    Jbb = np.zeros((3, 3))

    EIMatrices: List[np.ndarray] = []

    """
        Creates a 3 x 3 matrix
        | 1, 0, 0 |
        | 0, 1, 0 |
        | 0, 0, 1 |

    """
    Id3 = np.eye(3)
    
    tt_o_tt = np.zeros((3, 3))
    tilde_d1_3d = np.zeros((3, 3))
    tilde_d2_3d = np.zeros((3, 3))
    tf_c_d2t_o_tt = np.zeros((3, 3))
    tt_o_tf_c_d2t = np.zeros((3, 3))
    kb_o_d2e = np.zeros((3, 3))
    d2e_o_kb = np.zeros((3, 3))
    D2kappa1De2 = np.zeros((3, 3))
    te_c_d2t_o_tt = np.zeros((3, 3))
    tt_o_te_c_d2t = np.zeros((3, 3))
    kb_o_d2f = np.zeros((3, 3))
    d2f_o_kb = np.zeros((3, 3))
    D2kappa1Df2 = np.zeros((3, 3))
    D2kappa1DeDf = np.zeros((3, 3))
    D2kappa1DfDe = np.zeros((3, 3))
    tf_c_d1t_o_tt = np.zeros((3, 3))
    tt_o_tf_c_d1t = np.zeros((3, 3))
    kb_o_d1e = np.zeros((3, 3))
    d1e_o_kb = np.zeros((3, 3))
    D2kappa2De2 = np.zeros((3, 3))
    te_c_d1t_o_tt = np.zeros((3, 3))
    tt_o_te_c_d1t = np.zeros((3, 3))
    kb_o_d1f = np.zeros((3, 3))
    d1f_o_kb = np.zeros((3, 3))
    D2kappa2Df2 = np.zeros((3, 3))
    D2kappa2DeDf = np.zeros((3, 3))
    D2kappa2DfDe= np.zeros((3, 3))
    
    def __init__(self, soft_robots: SoftRobots):
        super().__init__(soft_robots)

        for limb in soft_robots.limbs:
            EI = limb.EI
            print(EI)


    def compute_force(self, dt):
        return super().compute_force(dt)
    
    def compute_force_and_jacobian(self, dt):
        return super().compute_force_and_jacobian(dt)
