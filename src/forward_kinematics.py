import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class JointState:
    """Inputs: Joint angles in degrees (as per standard reading humans)."""
    j1: float
    j2: float
    j3: float
    j4: float

class ForwardKinematics:
    """
    Calculates the 3D position of the end-effector using 
    Homogeneous Transformation Matrices (Denavit-Hartenberg Convention).
    """
    
    def __init__(self, link_length: float = 1.0):
        """
        Args:
            link_length (float): Length of L' (given as 1m in problem).
        """
        self.L = link_length

    def _get_dh_matrix(self, theta: float, d: float, a: float, alpha: float) -> np.matrix:
        """
        Creates the 4x4 Homogeneous Transformation Matrix for a single link
        using Modified Denavit-Hartenberg parameters.
        
        Math Concept:
        T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
        """
        # Convert angle to radians
        rad = np.radians(theta)
        alph_rad = np.radians(alpha)
        
        c = np.cos(rad)
        s = np.sin(rad)
        ca = np.cos(alph_rad)
        sa = np.sin(alph_rad)

        # The Standard DH Matrix
        # [ Rotation (3x3)   Translation (3x1) ]
        # [ 0        0       0       1         ]
        return np.matrix([
            [c, -s*ca,  s*sa, a * c],
            [s,  c*ca, -c*sa, a * s],
            [0,  sa,    ca,   d    ],
            [0,  0,     0,    1    ]
        ])

    def solve(self, joints: JointState) -> Tuple[float, float, float]:
        """
        Computes the (x, y, z) coordinates of the end-effector.
        """
        # --- THE CORE ALGORITHM (Linear Algebra) ---
        # Based on the problem description: "Axis of each joint is perpendicular to previous"
        # We construct the DH Parameter Table: [theta, d, a, alpha]
        # theta: joint angle (variable)
        # d: offset along previous z (0 for this diagram)
        # a: link length (L)
        # alpha: twist angle (90 degrees because axes are perpendicular)
        
        dh_table = [
            # Link 1: J1 (X-axis plane) -> J2 (Perpendicular)
            # theta, d, a, alpha
            (joints.j1, 0, self.L, 90), 
            
            # Link 2: J2 -> J3 (Perpendicular again)
            (joints.j2, 0, self.L, -90),
            
            # Link 3: J3 -> J4 (Perpendicular)
            (joints.j3, 0, self.L, 90),
            
            # Link 4: J4 -> End Effector (No more twist needed for position)
            (joints.j4, 0, self.L, 0)
        ]

        # Initialize Identity Matrix (Base Frame)
        T_total = np.matrix(np.identity(4))

        # Chain Multiplication: T_0_n = T_01 * T_12 * ... * T_n
        for params in dh_table:
            theta, d, a, alpha = params
            T_i = self._get_dh_matrix(theta, d, a, alpha)
            T_total = T_total * T_i

        # The End-Effector position is the Translation Column (Top-Right 3x1)
        # Matrix Structure:
        # [ r00 r01 r02  X ]
        # [ r10 r11 r12  Y ]
        # [ r20 r21 r22  Z ]
        # [  0   0   0   1 ]
        
        x = T_total[0, 3]
        y = T_total[1, 3]
        z = T_total[2, 3]

        return (x, y, z)

# --- EXECUTION ---
if __name__ == "__main__":
    # Test Case: All zeros
    # If all angles are 0, the arm should be fully extended along the X-axis (conceptually)
    fk = ForwardKinematics(link_length=1.0)
    joints = JointState(j1=0, j2=0, j3=0, j4=0)
    
    pos = fk.solve(joints)
    print(f"Joints: {joints}")
    print(f"End Effector Position (x, y, z): {round(pos[0], 2)}, {round(pos[1], 2)}, {round(pos[2], 2)}")
