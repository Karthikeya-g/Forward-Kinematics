from src import ForwardKinematics, JointState
import numpy as np

def test_case(name, j1, j2, j3, j4, expected_pos):
    """
    Helper function to run a test and print the result.
    """
    fk = ForwardKinematics(link_length=1.0)
    joints = JointState(j1, j2, j3, j4)
    result = fk.solve(joints)
    
    # Check if result matches expected (allowing for small math errors)
    is_correct = np.allclose(result, expected_pos, atol=1e-2)
    
    status = "✅ PASS" if is_correct else "❌ FAIL"
    print(f"{status} | {name}")
    print(f"   Input: {joints}")
    print(f"   Output: {np.round(result, 2)}")
    print(f"   Expected: {expected_pos}")
    print("-" * 30)

if __name__ == "__main__":
    print("--- 🤖 ROBOT ARM VERIFICATION ---")

    # TEST 1: The "Zero" Pose
    # All links straight out along X-axis.
    test_case("Zero Pose (Straight Out)", 
              0, 0, 0, 0, 
              (4.0, 0.0, 0.0))

    # TEST 2: The "Upward" Pose 
    # Base rotates 90. Whole arm points along Y.
    test_case("90 Deg Base Rotation", 
              90, 0, 0, 0, 
              (0.0, 4.0, 0.0))

    # TEST 3: The "Cobra" Pose
    # J1=0 (Face X), J2=90 (Turn Up).
    # L1 (1m) + L2,L3,L4 (3m Up)
    test_case("Cobra Pose (Bend Up)", 
              0, 90, 0, 0, 
              (1.0, 0.0, 3.0))

    # TEST 4: The "Corkscrew" Pose (CORRECTED)
    # Inputs: 0, 90, 90, 0
    # Logic:
    # 1. Link 1 goes Forward (X) -> Pos (1, 0, 0)
    # 2. J2 (Perpendicular) rotates 90 -> Link 2 goes Up (Z) -> Pos (1, 0, 1)
    # 3. J3 (Perpendicular) rotates 90 -> Link 3 goes Left (-Y) -> Pos (1, -1, 1)
    # 4. J4 (0) -> Link 4 continues Left (-Y) -> Pos (1, -2, 1)
    test_case("Corkscrew (3D Turn)", 
              0, 90, 90, 0, 
              (1.0, -2.0, 1.0))
