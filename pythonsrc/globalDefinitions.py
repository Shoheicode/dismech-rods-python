from enum import Enum
import numpy as np

class IntegratorMethod(Enum):
    FORWARD_EULER = "FORWARD_EULER"
    VERLET_POSITION = "VERLET_POSITION"
    BACKWARD_EULER = "BACKWARD_EULER"
    IMPLICIT_MIDPOINT = "IMPLICIT_MIDPOINT"

