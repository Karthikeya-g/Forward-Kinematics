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
    # We use np.allclose for float comparison
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
    # If all angles are 0, the arm extends straight along X.
    # Length = L + L + L + L = 4.0 meters
    test_case("Zero Pose (Straight Out)", 
              0, 0, 0, 0, 
              (4.0, 0.0, 0.0))

    # TEST 2: The "Upward" Pose (J1=90 degrees)
    # If J1 rotates 90 deg, the whole arm should point along Y.
    test_case("90 Deg Base Rotation", 
              90, 0, 0, 0, 
              (0.0, 4.0, 0.0))

    # TEST 3: The "Cobra" Pose (J2=90 degrees)
    # J1=0 (Face X)
    # J2=90 (Bend Up). 
    # Link 1 is flat (1m). Links 2,3,4 go UP (3m).
    # Expected: X=1, Y=0, Z=3
    test_case("Cobra Pose (Bend Up)", 
              0, 90, 0, 0, 
              (1.0, 0.0, 3.0))

    # TEST 4: The "Square" Pose (J2=90, J3=90)
    # J1=0 (Face X)
    # J2=90 (Link 2 goes Up)
    # J3=90 (Link 3 goes Back towards -X)
    # J4=0 (Link 4 continues Back towards -X)
    # Link 1 (Forward 1m) -> Link 2 (Up 1m) -> Link 3 (Back 1m) -> Link 4 (Back 1m)
    # Net X = 1 - 1 - 1 = -1
    # Net Z = 1
    # Expected: X=-1, Y=0, Z=1
    test_case("Square Hook", 
              0, 90, 90, 0, 
              (-1.0, 0.0, 1.0))
